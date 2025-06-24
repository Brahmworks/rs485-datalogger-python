#!/usr/bin/env python3
"""
Dual PID500-U Reader for COM6 and COM2
Reads from both devices simultaneously and displays data side by side
"""

import csv
import os
from datetime import datetime

import serial
import time
import threading
import queue
from datetime import datetime

class PID500UReader:
    def __init__(self, com_port, device_name, baud_rate=9600, slave_id=1):
        self.com_port = com_port
        self.device_name = device_name
        self.baud_rate = baud_rate
        self.slave_id = slave_id
        self.ser = None
        self.is_connected = False
        self.last_values = {}
        self.read_count = 0
        self.error_count = 0
        
    def connect(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(
                port=self.com_port,
                baudrate=self.baud_rate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1.0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            print(f"✓ {self.device_name} connected to {self.com_port}")
            time.sleep(0.5)
            self.is_connected = True
            return True
        except Exception as e:
            print(f"✗ {self.device_name} connection failed: {e}")
            self.is_connected = False
            return False
    
    def calculate_crc(self, data):
        """Calculate Modbus RTU CRC"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc
    
    def read_input_registers(self, start_address, count=1):
        """Read input registers (function code 0x04)"""
        if not self.ser or not self.ser.is_open:
            return None
            
        modbus_address = start_address - 30001
        
        frame = bytearray([
            self.slave_id, 0x04,
            (modbus_address >> 8) & 0xFF, modbus_address & 0xFF,
            (count >> 8) & 0xFF, count & 0xFF
        ])
        
        crc = self.calculate_crc(frame)
        frame.extend([crc & 0xFF, (crc >> 8) & 0xFF])
        
        try:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.write(frame)
            time.sleep(0.1)
            
            expected_length = 5 + (count * 2)
            response = self.ser.read(expected_length)
            
            if len(response) < expected_length:
                return None
                
            if len(response) >= 2 and response[1] & 0x80:
                return None
                
            if response[0] != self.slave_id or response[1] != 0x04:
                return None
                
            data = []
            for i in range(count):
                high_byte = response[3 + (i * 2)]
                low_byte = response[4 + (i * 2)]
                raw_value = (high_byte << 8) | low_byte
                data.append(raw_value)
            
            return data if count > 1 else data[0]
            
        except Exception as e:
            return None
    
    def read_holding_registers(self, start_address, count=1):
        """Read holding registers (function code 0x03)"""
        if not self.ser or not self.ser.is_open:
            return None
            
        modbus_address = start_address - 40001
        
        frame = bytearray([
            self.slave_id, 0x03,
            (modbus_address >> 8) & 0xFF, modbus_address & 0xFF,
            (count >> 8) & 0xFF, count & 0xFF
        ])
        
        crc = self.calculate_crc(frame)
        frame.extend([crc & 0xFF, (crc >> 8) & 0xFF])
        
        try:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.write(frame)
            time.sleep(0.1)
            
            expected_length = 5 + (count * 2)
            response = self.ser.read(expected_length)
            
            if len(response) < expected_length:
                return None
                
            if len(response) >= 2 and response[1] & 0x80:
                return None
                
            if response[0] != self.slave_id or response[1] != 0x03:
                return None
                
            data = []
            for i in range(count):
                high_byte = response[3 + (i * 2)]
                low_byte = response[4 + (i * 2)]
                raw_value = (high_byte << 8) | low_byte
                data.append(raw_value)
            
            return data if count > 1 else data[0]
            
        except Exception as e:
            return None
    
    def convert_signed_int(self, value):
        """Convert unsigned 16-bit to signed"""
        if value > 32767:
            return value - 65536
        return value
    
    def convert_dint_to_float(self, high_reg, low_reg):
        """Convert two 16-bit registers to 32-bit signed integer"""
        dint_value = (high_reg << 16) | low_reg
        if dint_value > 2147483647:
            dint_value -= 4294967296
        return dint_value / 10.0
    
    def read_all_values(self):
        """Read all PID values in one go"""
        values = {
            'process_value': None,
            'setpoint': None,
            'output_percentage': None,
            'output_type': None,
            'status': 'Error'
        }
        
        try:
            # Read Process Value (PV)
            pv_raw = self.read_input_registers(30011)
            if pv_raw is not None:
                values['process_value'] = self.convert_signed_int(pv_raw) / 10.0
            
            # Read Setpoint (SP)
            set1_high = self.read_holding_registers(40032)
            set1_low = self.read_holding_registers(40033)
            if set1_high is not None and set1_low is not None:
                values['setpoint'] = self.convert_dint_to_float(set1_high, set1_low)
            
            # Read Output
            output_raw = self.read_input_registers(30014)
            if output_raw is not None:
                values['output_percentage'] = self.convert_signed_int(output_raw)
            
            # Read Output Type
            status_raw = self.read_input_registers(30012)
            if status_raw is not None:
                status_map = {0: "SSR", 1: "Relay", 2: "AOUT"}
                values['output_type'] = status_map.get(status_raw, f"Unknown({status_raw})")
            
            # Update status
            if values['process_value'] is not None:
                values['status'] = 'OK'
                self.last_values = values.copy()
                self.read_count += 1
            else:
                values['status'] = 'Error'
                self.error_count += 1
                
        except Exception as e:
            values['status'] = f'Error: {str(e)}'
            self.error_count += 1
            
        return values
    
    def disconnect(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.is_connected = False

class DualPIDMonitor:
    def __init__(self):
        self.device1 = PID500UReader('COM6', 'PID Controller 1')
        self.device2 = PID500UReader('COM2', 'PID Controller 2')
        self.running = False
        self.data_queue = queue.Queue()
        
    def connect_devices(self):
        """Connect both devices"""
        print("Connecting to devices...")
        success1 = self.device1.connect()
        success2 = self.device2.connect()
        
        if success1 and success2:
            print("✓ Both devices connected successfully!")
            return True
        elif success1 or success2:
            print("⚠ Only one device connected")
            return True
        else:
            print("✗ Failed to connect to any device")
            return False
    
    def read_device_thread(self, device):
        """Thread function to read from one device"""
        while self.running:
            try:
                if device.is_connected:
                    values = device.read_all_values()
                    
                    data_packet = {
                        'device_name': device.device_name,
                        'com_port': device.com_port,
                        'values': values,
                        'timestamp': datetime.now(),
                        'read_count': device.read_count,
                        'error_count': device.error_count
                    }
                    
                    self.data_queue.put(data_packet)
                
                time.sleep(1)  # Read every second
                
            except Exception as e:
                print(f"Error in {device.device_name} thread: {e}")
                time.sleep(2)
    
    def start_monitoring(self):
        """Start monitoring both devices"""
        self.running = True
        
        # Start threads for both devices
        self.thread1 = threading.Thread(target=self.read_device_thread, args=(self.device1,), daemon=True)
        self.thread2 = threading.Thread(target=self.read_device_thread, args=(self.device2,), daemon=True)
        
        self.thread1.start()
        self.thread2.start()
        
        print("✓ Started monitoring both devices")
    
    def stop_monitoring(self):
        """Stop monitoring"""
        self.running = False
        self.device1.disconnect()
        self.device2.disconnect()
        print("✓ Stopped monitoring")
    
    def get_latest_data(self):
        """Get latest data from both devices"""
        device_data = {}
        
        # Collect all recent data
        try:
            while True:
                try:
                    data = self.data_queue.get_nowait()
                    device_data[data['com_port']] = data
                except queue.Empty:
                    break
        except:
            pass
            
        return device_data

def main():
    print("=" * 60)
    print("    DUAL PID500-U CONTROLLER MONITOR")
    print("    COM6 & COM2 - Simultaneous Reading")
    print("=" * 60)

    monitor = DualPIDMonitor()

    if not monitor.connect_devices():
        print("\nTroubleshooting steps:")
        print("1. Check that devices are on COM6 and COM2")
        print("2. Verify RS-485 wiring and termination")
        print("3. Ensure both devices are powered")
        print("4. Check Modbus settings (9600 baud, slave ID 1)")
        input("\nPress Enter to exit...")
        return

    # Create CSV
    start_time = datetime.now()
    filename_time = start_time.strftime("%d-%m-%Y %H-%M")
    os.makedirs("logs", exist_ok=True)
    csv_path = os.path.join("logs", f"{filename_time}.csv")

    with open(csv_path, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Controller 1 - PV", "Controller 1 - SP", "Controller 2 - PV", "Controller 2 - SP"])

        try:
            monitor.start_monitoring()

            print("\n" + "=" * 80)
            print("LIVE DATA - Press Ctrl+C to stop")
            print("=" * 80)

            while True:
                device_data = monitor.get_latest_data()
                if device_data:
                    now = datetime.now()
                    print(f"\n{now.strftime('%Y-%m-%d %H:%M:%S')}")
                    print("-" * 80)
                    print(f"{'COM6 - PID Controller 1':<40} {'COM2 - PID Controller 2':<40}")
                    print("-" * 80)

                    data1 = device_data.get('COM6', {})
                    data2 = device_data.get('COM2', {})

                    values1 = data1.get('values', {})
                    values2 = data2.get('values', {})

                    pv1 = values1.get('process_value')
                    sp1 = values1.get('setpoint')
                    pv2 = values2.get('process_value')
                    sp2 = values2.get('setpoint')

                    pv1_str = f"{pv1:.1f}°C" if pv1 is not None else "Error"
                    pv2_str = f"{pv2:.1f}°C" if pv2 is not None else "Error"
                    sp1_str = f"{sp1:.1f}°C" if sp1 is not None else "Error"
                    sp2_str = f"{sp2:.1f}°C" if sp2 is not None else "Error"

                    print(f"Process Value: {pv1_str:<30} Process Value: {pv2_str}")
                    print(f"Setpoint:      {sp1_str:<30} Setpoint:      {sp2_str}")

                    out1 = values1.get('output_percentage')
                    out2 = values2.get('output_percentage')
                    out1_str = f"{out1:.1f}%" if out1 is not None else "Error"
                    out2_str = f"{out2:.1f}%" if out2 is not None else "Error"

                    print(f"Output:        {out1_str:<30} Output:        {out2_str}")
                    print(f"Output Type:   {values1.get('output_type', 'Unknown'):<30} Output Type:   {values2.get('output_type', 'Unknown')}")
                    print(f"Status:        {values1.get('status', 'Unknown'):<30} Status:        {values2.get('status', 'Unknown')}")
                    print(f"Reads/Errors:  {data1.get('read_count', 0)}/{data1.get('error_count', 0):<27} Reads/Errors:  {data2.get('read_count', 0)}/{data2.get('error_count', 0)}")
                    print("-" * 80)

                    writer.writerow([
                        now.strftime("%Y-%m-%d %H:%M:%S"),
                        f"{pv1:.1f}" if pv1 is not None else "",
                        f"{sp1:.1f}" if sp1 is not None else "",
                        f"{pv2:.1f}" if pv2 is not None else "",
                        f"{sp2:.1f}" if sp2 is not None else ""
                    ])
                    file.flush()

                time.sleep(2)

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            monitor.stop_monitoring()
            print(f"\nData saved to: {csv_path}")
            print("Program ended")

if __name__ == "__main__":
    main()
