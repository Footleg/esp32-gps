import asyncio
from os import rename, dupterm
from machine import Pin, reset
from sys import print_exception
from time import sleep_ms
from net import Net
import config as cfg
from devices import Logger
try:
    from debug import DEBUG
except ImportError:
    DEBUG=False

# Wait 2 seconds to allow time to stop script after reset before REPL is disabled
#sleep_ms(2000)   

# Disable REPL on UART0 to free it for GPS/Serial use 
# (required for ESP32-C3 and similar boards with Integrated USB-Serial-JTAG Controller, 
# optional for others)
#dupterm(None, 0)

log = Logger.getLogger().log


class ESP32GPS():

    def __init__(self):
        self.net = None
        self.blue = None
        self.gps = None
        self.irq_event = asyncio.ThreadSafeFlag()
        self.espnow_event = asyncio.ThreadSafeFlag()
        self.shutdown_event = asyncio.Event()
        self.serial = None
        self.ntrip_caster = None
        self.ntrip_server = None
        self.ntrip_client = None
        self.tasks = []
        self.shell_callbacks = {}
        self._gps_rx_buf    = b""   # buffer for data coming from the GNSS UART
        self.statusMsg = "Booting up"

    def gps_reset(self):
        if (
            hasattr(cfg, "ENABLE_GPS_RESET") and
            (pin := getattr(cfg, "GPS_RESET_PIN", None))
        ):
            log(f"Resetting GPS device via pin: {pin}")
            reset_pin = Pin(pin, Pin.OUT)
            # Default to resetting by going 'high'
            reset_val = 1
            # Otherwise, reset by going 'low'
            if getattr(cfg, "GPS_RESET_MODE", "high") != "high":
                reset_val = 0

            # Sset reset value
            reset_pin.value(reset_val)
            sleep_ms(100)
            # Revert to inverse of reset value
            reset_pin.value(not reset_val)


    def hard_reset(self):
        """Reset device and GPS device."""

        self.gps_reset()
        # Reset esp32 device
        reset()


    def setup_gps(self):
        log("Enabling GPS device.")
        from devices import GPS
        try:
            self.gps = GPS(uart=cfg.GPS_UART, baudrate=cfg.GPS_BAUD_RATE, tx=cfg.GPS_TX_PIN, rx=cfg.GPS_RX_PIN)
        except (AttributeError, ValueError, OSError) as e:
            log(f"Error setting up GPS: {e}")
            self.statusMsg = "Failed to set up GPS"
            return False
        if hasattr(self.gps, "uart"):
            if (cmds := getattr(cfg, "GPS_SETUP_COMMANDS", None)):
                # Prefix used to filter response messages
                prefix = getattr(cfg, "GPS_SETUP_RESPONSE_PREFIX", "")
                for cmd in cmds:
                    self.gps.write_nmea(cmd, prefix)
                if hasattr(cfg, "GPS_SETUP_COMMANDS_RESET"):
                    self.gps_reset()
        return True

    def setup_serial(self):
        from devices import Serial
        log_serial = getattr(cfg, "LOG_TO_SERIAL", False)
        try:
            self.serial = Serial(uart=cfg.SERIAL_UART, baudrate=cfg.SERIAL_BAUD_RATE, tx=cfg.SERIAL_TX_PIN, rx=cfg.SERIAL_RX_PIN, log_serial=log_serial)
        except AttributeError:
            # No config options passed in
            pass


    def setup_networks(self):
        txpower = getattr(cfg, "WIFI_TXPOWER", None)
        self.net = Net(txpower=txpower)
        # Note: We start wifi first, as this will define the channel to be used.
        # Wifi connections also enable power management, which espnow startup will later disable.
        # See: https://docs.micropython.org/en/latest/library/espnow.html#espnow-and-wifi-operation
        if ((ssid := getattr(cfg, "WIFI_SSID", None)) and (psk := getattr(cfg, "WIFI_PSK"))):
            self.net.enable_wifi(ssid=cfg.WIFI_SSID, key=cfg.WIFI_PSK)
        # Start ESPNow if peers provided
        peers = getattr(cfg, "ESPNOW_PEERS", set())
        if (espnow_mode := getattr(cfg, "ESPNOW_MODE", None)):
            self.net.enable_espnow(peers=peers)
            if hasattr(cfg, "ESPNOW_DISCOVER_PEERS"):
                # Regularly broadcast presence for peer discovery
                self.tasks.append(asyncio.create_task(self.net.espnow_broadcast()))
                if espnow_mode == "sender":
                    # Read occasionally to look for peers
                    self.tasks.append(asyncio.create_task(self.net.espnow_find_peers()))

    def esp32_write_data(self, value):
        """Callback to run if device is written to (BLE, Serial)"""
        self.gps.uart.write(value)

    def split_nmea_sentences(self, buf: bytes) -> list[str]:
        """
        Extract all complete NMEA sentences from buffer.

        A valid sentence:
            • starts with b'$'
            • contains a '*' checksum delimiter
            • ends with CRLF (b'\r\n')
        The function returns a list of the complete NMEA sentences from the buffer
        (including the leading '$' but without the trailing CR/LF). 
        Any incomplete tail is left in the buffer to be consumed on the next call.
        """

        # Append the new data to existing data in the buffer
        self._gps_rx_buf += buf

        # Find end of the last complete nmea sentence in the buffer
        last_crlf = self._gps_rx_buf.rfind(b'\r\n')
        if last_crlf != -1:
            # All complete sentences up to that point
            chunk = self._gps_rx_buf[:last_crlf + 2]
            # Save the leftover (might be empty or a fragment of the next sentence)
            self._gps_rx_buf = self._gps_rx_buf[last_crlf + 2 :]
        else:
            # No complete sentences yet, keep everything in the buffer
            return []
        
        sentences = []
        # Keep looping while we can find a full "$…*XX\r\n" pattern
        while True:
            # Find the start of a sentence
            start = chunk.find(b"$")
            if start == -1:
                # No start marker – discard everything (garbage)
                return sentences

            # Look for the end marker after the start
            end = chunk.find(b"\r\n", start)
            if end == -1:
                # No CRLF yet → incomplete sentence, keep it for later
                break

            # Extract the candidate sentence (including the leading '$')
            candidate = chunk[start:end]          # b'$GNGGA,...*5A'
            # Verify that a checksum delimiter exists
            # TODO: Validate checksum here
            if b"*" in candidate:
                sentences.append(candidate.decode("utf-8", "ignore"))

            # Move the buffer past the processed sentence and continue
            chunk = chunk[end + 2 :]                # skip the "\r\n"

        return sentences

    async def ntrip_client_read(self):
        """Read data from NTRIP client and write to GPS device."""
        while True:
            data = await self.ntrip_client.iter_data()
            self.esp32_write_data(data)

    async def espnow_reader(self):
        """Read from ESPNow in async loop, and send for outputting."""
        discover_peers = getattr(cfg, "ESPNOW_DISCOVER_PEERS", False)
        while True:
            try:
                data = await self.net.espnow_recv(discover_peers=discover_peers)
                if data:
                    await self.gps_data(data)
            except Exception as e:
                print_exception(e)
            await asyncio.sleep(0)

    async def gps_reader(self):
        # FIXME: Move to code where this task is instantiated
        if ( hasattr(self.gps, "uart") and
            ("server" in getattr(cfg, "NTRIP_MODE", []) or
            getattr(cfg, "ESPNOW_MODE", None) == "sender" or
            hasattr(cfg, "ENABLE_SERIAL_CLIENT") or
            (self.blue and self.blue.is_connected()))
        ):
            while True:
                try:
                    data = self.gps.uart.read()
                    if data:
                        await self.gps_data(data)
                except Exception as e:
                    print_exception(e)
                await asyncio.sleep_ms(0)
        else:
            print("No GPS. Nothing to do!")
            

    async def gps_data(self, line):
        """Read GPS data and send to configured outputs.

        All exceptions are caught and logged to avoid crashing the main thread.

        NMEA sentences are sent to (if enabled): USB serial, Bluetooth, ESPNow and NTRIP server (only non-NMEA data).
        """
        if not line:
            return
        isNMEA = False
        # Handle NMEA sentences
        if line.startswith(b"$") and line.endswith(b"\r\n"):
            isNMEA = True
            if cfg.ENABLE_GPS and cfg.PQTMEPE_TO_GGST:
                if line.startswith(b"$GNRMC"):
                    # Extract UTC_TIME (as str) for use in GST sentence creation
                    self.gps.utc_time = line.split(b",",2)[1].decode("UTF-8")
                if line.startswith(b"$PQTMEPE"):
                    line = self.gps.pqtmepe_to_gst(line)
        try:
            if cfg.ENABLE_SERIAL_CLIENT:
                # Only send a line if the last transmit completed - avoid buffer overflow
                if hasattr(self.serial, "uart") and self.serial.uart.txdone():
                    self.serial.uart.write(line)
                    self.serial.uart.flush()
                elif self.serial.id == 0:
                    # For boards with an Integrated USB-Serial-JTAG Controller 
                    # (e.g. ESP32-C3), we can print directly to the console without 
                    # using a UART. We want to split the buffer into individual NMEA 
                    # sentences and send each one separately.
                    for sentence in self.split_nmea_sentences(line):
                        # Forward the complete NMEA sentence to the serial output
                        print(sentence)
        except Exception as e:
            log(f"[GPS DATA] USB serial send exception: {print_exception(e)}")
        try:
            if cfg.ENABLE_BLUETOOTH and self.blue.is_connected():
                self.blue.send(line)
        except Exception as e:
            log(f"[GPS DATA] BT send exception: {print_exception(e)}")

        try:
            if self.net.espnow_connected and cfg.ESPNOW_MODE == "sender":
                await self.net.espnow_sendall(line)
        except Exception as e:
            log(f"[GPS DATA] ESPNow send exception: {print_exception(e)}")

        try:
            # Don't sent NMEA sentences to NTRIP server
            if not isNMEA and self.ntrip_server:
                await self.ntrip_server.send_data(line)
        except Exception as e:
            log(f"[GPS DATA] NTRIP server send exception: {print_exception(e)}")
        # Settle
        await asyncio.sleep(0)


    async def serial_to_gps_nmea_forwarder(self):
        """
        Forward any NMEA sentences received over usb serial to the GPS module.

        The task keeps a small byte buffer because data may arrive in
        fragments. When a full line ending with CR/LF is assembled we validate 
        that it looks like an NMEA sentence before writing it to the GNSS UART.
        """
        # Check both serial interfaces are set up before starting task.
        if not (self.serial and hasattr(self.serial, "uart") and
                self.gps and hasattr(self.gps, "uart")):
            return

        # Buffer for bytes that have not yet formed a complete line.
        rx_buf = b""

        while True:
            try:
                data = self.serial.uart.read()
                if data:
                    # Append new bytes to the buffer.
                    rx_buf += data

                    # Process every complete line that ends with CR/LF.
                    while b"\r\n" in rx_buf:
                        line, rx_buf = rx_buf.split(b"\r\n", 1)
                        line += b"\r\n"          # restore terminator for validation

                        # -------------------------------------------------
                        # NMEA validation – keep it simple:
                        #   * starts with '$'
                        #   * contains a '*' before the final CR/LF
                        #   * has at least one character between '$' and '*'
                        # -------------------------------------------------
                        if (
                            line.startswith(b"$") 
                            and len(line) > 4 
                            and b"*" in line[3:]
                        ):
                            # Forward the raw sentence to the gps module.
                            self.gps.uart.write(line)
                        else:
                            # Not an NMEA sentence – ignore it.
                            pass
                # If no data was available, just yield to the event loop.
            except Exception as e:
                log(f"[USB→GPS] read/write error: {e}")

            await asyncio.sleep_ms(0)   # cooperative pause


    def setup_shell_callbacks(self):
        for cmd in ["CFG", "GPS", "RESET", "RESETGPS"]:
            self.shell_callbacks[cmd] = getattr(self, f"cb_{cmd}")

    # Callback functions for shell remote commands
    def cb_CFG(self, opts):
        """Report current config or update config.py on the device."""
        conf_dict = { k: getattr(cfg, k) for k in dir(cfg) if k.isupper() }
        if not opts:
            # Return current config
            return "\n".join([f"{k}={v}" for k, v in conf_dict.items()])

        try:
            key, val = opts.split("=")
            # Remove whitespace from cfg args
            key = key.strip()
            # Try to convert the value string into a python object
            val = eval(val.strip())
            conf_dict[key] = val
        except ValueError as e:
            return(f"Invalid config. Syntax: CFG KEY=val (strings must be quoted). {e}")
        # Get all current config attrs, and add/update this one
        try:
            # Write a temporary config file, then replace original
            with open("config.py.tmp", "w") as conf_f:
                for k, v in conf_dict.items():
                    conf_f.write(f"{k} = {repr(v)}\n")
            rename("config.py.tmp", "config.py")
            return(f"Updated config: {key}={repr(val)}")
        except OSError as e:
            return(f"Unable to save config to file: {e}")

    def cb_GPS(self, opts):
        """Write a command to the GPS device."""
        if hasattr(self.gps, "uart"):
            # Prefix used to filter response messages
            prefix = getattr(cfg, "GPS_SETUP_RESPONSE_PREFIX", "")
            # Return the GPS response output
            return self.gps.write_nmea(opts, prefix)

    def cb_RESETGPS(self, opts):
        """Reset just the GPS device."""
        self.gps_reset()
        return("GPS device reset.")

    def cb_RESET(self, opts):
        """Hard reset the device."""
        self.hard_reset()

    async def run(self):
        """Start various long-running async processes.

        There are 2 conditions which affect which services to start:
        1. GPS data, sourced either from a GPS device, or ESPNOW receiver.
        2. Wifi connection.

        Data source is needed for:
        a. Bluetooth.
        b. Serial output
        c. NTRIP Server.

        Wifi is needed for:
        a. NTRIP services (caster, server, client)
        """

        # Start serial early, as logs may be redirected to it.
        if getattr(cfg, "ENABLE_SERIAL_CLIENT", None):
            self.setup_serial()
            if hasattr(self.serial, "uart"):
                log(f"Serial output enabled (UART{self.serial.id})")
            elif self.serial.id == 0:
                log("Serial output via Integrated USB-Serial-JTAG Controller")
            else:
                # Serial setup didn't create uart for some reason, so turn off serial logging
                cfg.ENABLE_SERIAL_CLIENT = False

        # Set up wifi
        self.setup_networks()

        # Set up remote shell
        if hasattr(cfg, "ENABLE_SHELL"):
            from shell import Shell
            self.setup_shell_callbacks()
            kwargs = {}
            if (addr := getattr(cfg, "SHELL_BIND_ADDRESS", None)):
                kwargs["bind_address"] = addr
            if (port := getattr(cfg, "SHELL_BIND_PORT", None)):
                kwargs["bind_port"] = port
            if (passwd := getattr(cfg, "SHELL_PASSWORD", None)):
                kwargs["password"] = passwd
            sh = Shell(callbacks=self.shell_callbacks, **kwargs)
            self.tasks.append(asyncio.create_task(sh.run()))

        # Expect to receive gps data (from device, or ESPNOW)
        gps_setup_success = True
        espnow_mode = getattr(cfg, "ESPNOW_MODE", None)
        if cfg.ENABLE_GPS:
            gps_setup_success = self.setup_gps()
            print(f"GPS setup outcome: {gps_setup_success}")
            print(f"Status: {self.statusMsg}")
            if gps_setup_success:
                self.tasks.append(asyncio.create_task(self.gps_reader()))
                # Now start usb serial to gps forwarding as both serial interfaces should be up by now
                self.tasks.append(asyncio.create_task(self.serial_to_gps_nmea_forwarder()))
                # sender goes with GPS device
                if espnow_mode == "sender":
                    log("ESPNow: sender mode.")
            else:
                log("No GPS device found. Serial, Bluetooth and NTRIP server output will be disabled.")
        elif espnow_mode == "receiver":
            log("ESPNow: receiver mode.")
            self.tasks.append(asyncio.create_task(self.espnow_reader()))
        else:
            log("No GPS source available. Serial, Bluetooth and NTRIP server output will be disabled.")
            gps_setup_success = False


        # No point enabling bluetooth if no GPS data to send
        if gps_setup_success and cfg.ENABLE_BLUETOOTH:
            from blue import Blue
            log("Enabling Bluetooth")
            self.blue = Blue(name=cfg.DEVICE_NAME)
            # Set custom BLE write callback
            self.blue.write_callback = self.esp32_write_data

        # NTRIP needs a network connection
        if self.net and self.net.wifi_connected:
            if cfg.NTRIP_MODE:
                import ntrip
            if "caster" in cfg.NTRIP_MODE:
                self.ntrip_caster = ntrip.Caster(cfg.NTRIP_CASTER_BIND_ADDRESS, cfg.NTRIP_CASTER_BIND_PORT, cfg.NTRIP_SOURCETABLE, cfg.NTRIP_CLIENT_CREDENTIALS, cfg.NTRIP_SERVER_CREDENTIALS)
                self.tasks.append(asyncio.create_task(self.ntrip_caster.run()))
                # Allow Caster to start before Server/Client
                await asyncio.sleep(2)
            if gps_setup_success and "server" in cfg.NTRIP_MODE:
                self.ntrip_server = ntrip.Server(cfg.NTRIP_CASTER, cfg.NTRIP_PORT, cfg.NTRIP_MOUNT, cfg.NTRIP_SERVER_CREDENTIALS)
                self.tasks.append(asyncio.create_task(self.ntrip_server.run()))
            if cfg.ENABLE_GPS and "client" in cfg.NTRIP_MODE:
                self.ntrip_client = ntrip.Client(cfg.NTRIP_CASTER, cfg.NTRIP_PORT, cfg.NTRIP_MOUNT, cfg.NTRIP_CLIENT_CREDENTIALS)
                self.tasks.append(asyncio.create_task(self.ntrip_client.run()))
                self.tasks.append(asyncio.create_task(self.ntrip_client_read()))

        # Wait for shutdown_event signal
        await self.shutdown_event.wait()

    async def shutdown(self):
        """Clean up background processes, handlers etc on exit."""
        # Stop bluetooth irq handling
        if hasattr(self.blue, "ble"):
            self.blue.ble.irq(None)

        # Signal ntrip_caster to shutdown
        if hasattr(self.ntrip_caster, "shutdown"):
            await self.ntrip_caster.shutdown()

        # Clean up self
        for task in self.tasks:
            try:
                task.cancel()
            except:
                pass

        # Wait for tasks to exit
        await asyncio.gather(*self.tasks, return_exceptions=True)

if __name__ == "__main__":
    e32gps = ESP32GPS()
    log("Starting firmware...")
    try:
        loop = asyncio.get_event_loop()
        loop.run_until_complete(e32gps.run())

        # The background tasks in e32gps should run forever (or raise exceptions).
        # We only reach here if the event loop exits cleanly - i.e no background tasks.
        log("Exited - nothing to do.")
        log("Enable at least one long-running process in your configuration: (GPS, ESPNow Receiver, NTRIP)")

        # Clean up hanging IRQ etc
        loop.run_until_complete(e32gps.shutdown())

    except (KeyboardInterrupt, Exception) as e:
        e32gps.shutdown_event.set()
        loop.run_until_complete(e32gps.shutdown())
        if isinstance(e, KeyboardInterrupt):
            log("Ctrl-C received - shutting down.")
        else:
            log("Unhandled exception - shutting down.")
            print_exception(e)
            if getattr(cfg, "CRASH_RESET", None):
                log("Hard resetting due to crash...")
                # Delay (to prevent restart tight loop, and give time to read the exception)
                sleep_ms(5000)
                reset()
