#!/usr/bin/env python3
"""
Android Device Information Reader using ADB
Reads CPU, GPU, RAM, device info and more from Android devices via ADB
"""

import subprocess
import json
import re
import time
from typing import Dict, List, Optional, Any
from datetime import datetime

class ADBDeviceInfo:
    def __init__(self, device_id: Optional[str] = None):
        """
        Initialize ADB Device Info reader
        
        Args:
            device_id: Specific device ID to target (optional)
        """
        self.device_id = device_id
        self.device_cmd = ["-s", device_id] if device_id else []
    
    def run_adb_command(self, command: List[str], timeout: int = 30) -> str:
        """Execute ADB command and return output with better error handling"""
        try:
            full_command = ["adb"] + self.device_cmd + command
            result = subprocess.run(
                full_command, 
                capture_output=True, 
                text=True, 
                timeout=timeout
            )
            if result.returncode != 0:
                # Don't print errors for expected failures (like missing files)
                if "No such file or directory" not in result.stderr:
                    print(f"ADB command failed: {' '.join(command)}")
                    print(f"Error: {result.stderr.strip()}")
                return ""
            return result.stdout.strip()
        except subprocess.TimeoutExpired:
            print(f"ADB command timed out: {' '.join(command)}")
            return ""
        except FileNotFoundError:
            print("ADB not found. Please install Android SDK platform tools.")
            return ""
        except Exception as e:
            print(f"Error running ADB command: {e}")
            return ""
    
    def get_connected_devices(self) -> List[Dict[str, str]]:
        """Get list of connected devices"""
        output = self.run_adb_command(["devices", "-l"])
        devices = []
        
        for line in output.split('\n')[1:]:  # Skip header
            if line.strip() and 'device' in line:
                parts = line.split()
                if len(parts) >= 2:
                    device_info = {
                        'id': parts[0],
                        'status': parts[1]
                    }
                    
                    # Parse additional info
                    for part in parts[2:]:
                        if ':' in part:
                            key, value = part.split(':', 1)
                            device_info[key] = value
                    
                    devices.append(device_info)
        
        return devices
    
    def get_device_properties(self) -> Dict[str, str]:
        """Get device build properties"""
        output = self.run_adb_command(["shell", "getprop"])
        properties = {}
        
        for line in output.split('\n'):
            match = re.match(r'\[([^\]]+)\]: \[(.*)\]', line)
            if match:
                key, value = match.groups()
                properties[key] = value
        
        return properties
    
    def get_basic_device_info(self) -> Dict[str, Any]:
        """Get basic device information"""
        props = self.get_device_properties()
        
        return {
            'manufacturer': props.get('ro.product.manufacturer', 'Unknown'),
            'brand': props.get('ro.product.brand', 'Unknown'),
            'model': props.get('ro.product.model', 'Unknown'),
            'device': props.get('ro.product.device', 'Unknown'),
            'android_version': props.get('ro.build.version.release', 'Unknown'),
            'api_level': props.get('ro.build.version.sdk', 'Unknown'),
            'build_number': props.get('ro.build.display.id', 'Unknown'),
            'security_patch': props.get('ro.build.version.security_patch', 'Unknown'),
            'bootloader': props.get('ro.bootloader', 'Unknown'),
            'serial': props.get('ro.serialno', 'Unknown'),
            'fingerprint': props.get('ro.build.fingerprint', 'Unknown'),
            'incremental': props.get('ro.build.version.incremental', 'Unknown'),
            'abi': props.get('ro.product.cpu.abi', 'Unknown'),
            'abi2': props.get('ro.product.cpu.abi2', 'Unknown')
        }
    
    def get_cpu_info(self) -> Dict[str, Any]:
        """Get CPU information"""
        # Get CPU info from /proc/cpuinfo
        cpuinfo = self.run_adb_command(["shell", "cat", "/proc/cpuinfo"])
        
        cpu_data = {
            'cores': [],
            'architecture': 'Unknown',
            'features': [],
            'core_count': 0
        }
        
        current_core = {}
        core_count = 0
        
        for line in cpuinfo.split('\n'):
            if ':' in line:
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip()
                
                if key == 'processor':
                    if current_core:
                        cpu_data['cores'].append(current_core)
                    current_core = {'processor': value}
                    core_count += 1
                elif key in ['model name', 'Processor', 'Hardware']:
                    current_core['model'] = value
                elif key == 'cpu MHz':
                    current_core['frequency'] = f"{value} MHz"
                elif key == 'Features':
                    cpu_data['features'] = value.split()
                elif key == 'CPU architecture':
                    cpu_data['architecture'] = value
                elif key == 'CPU implementer':
                    current_core['implementer'] = value
                elif key == 'CPU part':
                    current_core['part'] = value
        
        if current_core:
            cpu_data['cores'].append(current_core)
        
        cpu_data['core_count'] = core_count
        
        # Get CPU frequencies for all cores
        freq_info = self.get_cpu_frequencies()
        cpu_data.update(freq_info)
        
        # Get CPU usage
        cpu_data['current_usage'] = self.get_cpu_usage()
        
        return cpu_data
    
    def get_cpu_frequencies(self) -> Dict[str, Any]:
        """Get CPU frequency information for all cores"""
        freq_info = {'frequencies': []}
        
        # Try to get frequency info for all cores
        for i in range(8):  # Most devices have max 8 cores
            max_freq = self.run_adb_command([
                "shell", 
                "cat", 
                f"/sys/devices/system/cpu/cpu{i}/cpufreq/cpuinfo_max_freq"
            ])
            
            min_freq = self.run_adb_command([
                "shell", 
                "cat", 
                f"/sys/devices/system/cpu/cpu{i}/cpufreq/cpuinfo_min_freq"
            ])
            
            current_freq = self.run_adb_command([
                "shell", 
                "cat", 
                f"/sys/devices/system/cpu/cpu{i}/cpufreq/scaling_cur_freq"
            ])
            
            if max_freq and max_freq.isdigit():
                core_freq = {
                    'core': i,
                    'max_frequency': f"{int(max_freq) // 1000} MHz"
                }
                
                if min_freq and min_freq.isdigit():
                    core_freq['min_frequency'] = f"{int(min_freq) // 1000} MHz"
                
                if current_freq and current_freq.isdigit():
                    core_freq['current_frequency'] = f"{int(current_freq) // 1000} MHz"
                
                freq_info['frequencies'].append(core_freq)
            else:
                break  # No more cores
        
        return freq_info
    
    def get_cpu_usage(self) -> Optional[str]:
        """Get current CPU usage"""
        # Read /proc/stat for CPU usage
        stat1 = self.run_adb_command(["shell", "cat", "/proc/stat"])
        if not stat1:
            return None
            
        time.sleep(1)  # Wait 1 second
        
        stat2 = self.run_adb_command(["shell", "cat", "/proc/stat"])
        if not stat2:
            return None
        
        try:
            # Parse CPU usage from /proc/stat
            line1 = stat1.split('\n')[0]
            line2 = stat2.split('\n')[0]
            
            values1 = [int(x) for x in line1.split()[1:]]
            values2 = [int(x) for x in line2.split()[1:]]
            
            # Calculate usage
            idle1, idle2 = values1[3], values2[3]
            total1 = sum(values1)
            total2 = sum(values2)
            
            idle_diff = idle2 - idle1
            total_diff = total2 - total1
            
            usage = 100.0 * (total_diff - idle_diff) / total_diff
            return f"{usage:.1f}%"
        except:
            return None
    
    def get_gpu_info(self) -> Dict[str, Any]:
        """Get GPU information"""
        gpu_info = {
            'renderer': 'Unknown',
            'vendor': 'Unknown',
            'version': 'Unknown',
            'extensions': []
        }
        
        # Method 1: Try dumpsys SurfaceFlinger
        dumpsys_output = self.run_adb_command(["shell", "dumpsys", "SurfaceFlinger"])
        
        for line in dumpsys_output.split('\n'):
            if 'GLES:' in line:
                gl_info = line.split('GLES:')[1].strip()
                parts = gl_info.split(',')
                if len(parts) >= 3:
                    gpu_info['vendor'] = parts[0].strip()
                    gpu_info['renderer'] = parts[1].strip()
                    gpu_info['version'] = parts[2].strip()
                break
            elif 'GL_EXTENSIONS:' in line:
                extensions = line.split('GL_EXTENSIONS:')[1].strip()
                gpu_info['extensions'] = extensions.split()[:10]  # First 10 extensions
        
        # Method 2: Try getprop for GPU properties
        props = self.get_device_properties()
        if gpu_info['renderer'] == 'Unknown':
            gpu_info['renderer'] = props.get('ro.hardware.egl', props.get('ro.hardware.vulkan', 'Unknown'))
        
        return gpu_info
    
    def get_memory_info(self) -> Dict[str, Any]:
        """Get memory information"""
        # Get memory info from /proc/meminfo
        meminfo = self.run_adb_command(["shell", "cat", "/proc/meminfo"])
        
        memory_data = {}
        
        for line in meminfo.split('\n'):
            if ':' in line:
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip()
                
                if key in ['MemTotal', 'MemFree', 'MemAvailable', 'Buffers', 'Cached', 'SwapTotal', 'SwapFree']:
                    if 'kB' in value:
                        kb_value = int(value.split()[0])
                        memory_data[key.lower()] = f"{kb_value // 1024} MB"
                    else:
                        memory_data[key.lower()] = value
        
        # Calculate memory usage percentage
        if 'memtotal' in memory_data and 'memavailable' in memory_data:
            try:
                total = int(memory_data['memtotal'].replace(' MB', ''))
                available = int(memory_data['memavailable'].replace(' MB', ''))
                used = total - available
                usage_percent = (used / total) * 100
                memory_data['memory_usage'] = f"{usage_percent:.1f}%"
            except:
                pass
        
        return memory_data
    
    def get_storage_info(self) -> Dict[str, Any]:
        """Get storage information"""
        df_output = self.run_adb_command(["shell", "df", "-h"])
        
        storage_data = {
            'partitions': [],
            'total_storage': None,
            'available_storage': None
        }
        
        for line in df_output.splitlines()[1:]:
            if line.strip():
                parts = line.split()
                if len(parts) >= 6:
                    filesystem = parts[0]
                    size = parts[1]
                    used = parts[2]
                    available = parts[3]
                    use_percentage = parts[4]
                    mounted_on = " ".join(parts[5:])

                    storage_data['partitions'].append({
                        'filesystem': filesystem,
                        'size': size,
                        'used': used,
                        'available': available,
                        'use_percentage': use_percentage,
                        'mounted_on': mounted_on
                    })
                    
                    # Get main storage info
                    if mounted_on == '/data':
                        storage_data['total_storage'] = size
                        storage_data['available_storage'] = available
        
        return storage_data
    
    def get_battery_info(self) -> Dict[str, Any]:
        """Get battery information"""
        battery_output = self.run_adb_command(["shell", "dumpsys", "battery"])
        
        battery_data = {}
        
        for line in battery_output.split('\n'):
            line = line.strip()
            if ':' in line and not line.startswith('Battery'):
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip()
                
                if key in ['level', 'scale', 'voltage', 'temperature', 'technology', 'status', 'health']:
                    battery_data[key] = value
        
        # Convert temperature to Celsius if it's in decidegrees
        if 'temperature' in battery_data:
            try:
                temp = int(battery_data['temperature'])
                if temp > 100:  # Likely in decidegrees Celsius
                    battery_data['temperature'] = f"{temp / 10:.1f}°C"
                else:
                    battery_data['temperature'] = f"{temp}°C"
            except:
                pass
        
        return battery_data
    
    def get_network_info(self) -> Dict[str, Any]:
        """Get network interface information"""
        ifconfig = self.run_adb_command(["shell", "ip", "addr", "show"])
        
        if not ifconfig:
            ifconfig = self.run_adb_command(["shell", "ifconfig"])
        
        network_data = {
            'interfaces': [],
            'active_connections': []
        }
        
        # Parse network interfaces
        if ifconfig:
            current_interface = None
            for line in ifconfig.split('\n'):
                if re.match(r'^\d+:', line):  # ip addr format
                    match = re.search(r'^\d+: ([^:@]+)', line)
                    if match:
                        current_interface = {
                            'name': match.group(1),
                            'addresses': []
                        }
                        network_data['interfaces'].append(current_interface)
                elif 'inet ' in line and current_interface:
                    match = re.search(r'inet ([0-9.]+)', line)
                    if match:
                        current_interface['addresses'].append(match.group(1))
        
        return network_data
    
    def get_wifi_info(self) -> Dict[str, str]:
        """Get WiFi SSID, signal strength, link speed"""
        wifi_output = self.run_adb_command(["shell", "dumpsys", "wifi"])
        wifi_data = {}
        
        for line in wifi_output.splitlines():
            line = line.strip()
            if "mWifiInfo" in line and "SSID" in line:
                # Extract SSID from mWifiInfo line
                ssid_match = re.search(r'SSID: ([^,]+)', line)
                if ssid_match:
                    wifi_data["ssid"] = ssid_match.group(1).strip()
                
                # Extract RSSI
                rssi_match = re.search(r'RSSI: (-?\d+)', line)
                if rssi_match:
                    wifi_data["signal_strength"] = f"{rssi_match.group(1)} dBm"
                
                # Extract link speed
                speed_match = re.search(r'Link speed: (\d+)', line)
                if speed_match:
                    wifi_data["link_speed"] = f"{speed_match.group(1)} Mbps"
                
                break
        
        return wifi_data
    
    def get_display_info(self) -> Dict[str, str]:
        """Get screen resolution, density, refresh rate, brightness"""
        display_data = {}
        
        # Resolution and density
        size = self.run_adb_command(["shell", "wm", "size"])
        density = self.run_adb_command(["shell", "wm", "density"])

        if size and "Physical size:" in size:
            display_data["resolution"] = size.replace("Physical size: ", "").strip()
        if density and "Physical density:" in density:
            display_data["density"] = density.replace("Physical density: ", "").strip()

        # Brightness
        brightness = self.run_adb_command(
            ["shell", "settings", "get", "system", "screen_brightness"]
        )
        if brightness.isdigit():
            brightness_percent = int(brightness) / 255 * 100
            display_data["brightness"] = f"{brightness} ({brightness_percent:.0f}%)"

        # Brightness mode
        mode = self.run_adb_command(
            ["shell", "settings", "get", "system", "screen_brightness_mode"]
        )
        if mode in ["0", "1"]:
            display_data["brightness_mode"] = "auto" if mode == "1" else "manual"

        # Screen timeout
        timeout = self.run_adb_command(
            ["shell", "settings", "get", "system", "screen_off_timeout"]
        )
        if timeout.isdigit():
            timeout_sec = int(timeout) // 1000
            display_data["screen_timeout"] = f"{timeout_sec}s"

        return display_data
    
    def get_audio_info(self) -> Dict[str, Any]:
        """Get volume levels and connected audio devices"""
        audio_output = self.run_adb_command(["shell", "dumpsys", "audio"])
        audio_data = {
            'volume_levels': {},
            'audio_devices': []
        }
        
        for line in audio_output.splitlines():
            # Volume levels
            if "- STREAM_" in line and "muted:" in line:
                stream_match = re.search(r'STREAM_(\w+).*?device:.*?volume:(\d+)', line)
                if stream_match:
                    stream_name = stream_match.group(1).lower()
                    volume = stream_match.group(2)
                    audio_data['volume_levels'][stream_name] = volume
            
            # Audio devices
            elif "AudioDeviceInfo:" in line:
                audio_data['audio_devices'].append(line.strip())
        
        return audio_data
    
    def get_camera_info(self) -> List[Dict[str, str]]:
        """Get available cameras and their characteristics"""
        cam_output = self.run_adb_command(["shell", "dumpsys", "media.camera"])
        cameras = []
        
        current_camera = None
        for line in cam_output.splitlines():
            if "Camera ID" in line:
                if current_camera:
                    cameras.append(current_camera)
                current_camera = {'id': line.strip()}
            elif current_camera and ("Resolution" in line or "FPS" in line):
                if 'details' not in current_camera:
                    current_camera['details'] = []
                current_camera['details'].append(line.strip())
        
        if current_camera:
            cameras.append(current_camera)
        
        return cameras
    
    def get_security_info(self) -> Dict[str, str]:
        """Get encryption, SELinux, security state"""
        security_data = {}
        
        # Encryption state
        crypto = self.run_adb_command(["shell", "getprop", "ro.crypto.state"])
        if crypto:
            security_data["encryption"] = crypto
            
        # SELinux status
        selinux = self.run_adb_command(["shell", "getenforce"])
        if selinux:
            security_data["selinux"] = selinux
            
        # Lock screen security
        lock_pattern = self.run_adb_command(
            ["shell", "settings", "get", "secure", "lock_pattern_enabled"]
        )
        if lock_pattern in ["0", "1"]:
            security_data["pattern_lock"] = "enabled" if lock_pattern == "1" else "disabled"
            
        # Developer options
        dev_options = self.run_adb_command(
            ["shell", "settings", "get", "global", "development_settings_enabled"]
        )
        if dev_options in ["0", "1"]:
            security_data["developer_options"] = "enabled" if dev_options == "1" else "disabled"
            
        # USB debugging
        usb_debug = self.run_adb_command(
            ["shell", "settings", "get", "global", "adb_enabled"]
        )
        if usb_debug in ["0", "1"]:
            security_data["usb_debugging"] = "enabled" if usb_debug == "1" else "disabled"
        
        return security_data
    
    def get_sensor_info(self) -> List[Dict[str, str]]:
        """Get available sensors with more details"""
        sensor_output = self.run_adb_command(["shell", "dumpsys", "sensorservice"])
        
        sensors = []
        
        for line in sensor_output.split('\n'):
            if 'Sensor[' in line:
                sensor_match = re.search(r'Sensor\[(\d+)\]\s+(.+)', line)
                if sensor_match:
                    sensor_id = sensor_match.group(1)
                    sensor_info = sensor_match.group(2).strip()
                    
                    sensors.append({
                        'id': sensor_id,
                        'name': sensor_info
                    })
        
        return sensors[:20]  # Limit for readability
    
    def get_system_uptime(self) -> Dict[str, str]:
        """Get system uptime and boot time"""
        uptime_output = self.run_adb_command(["shell", "cat", "/proc/uptime"])
        boot_time = self.run_adb_command(["shell", "getprop", "ro.runtime.firstboot"])
        
        uptime_data = {}
        
        if uptime_output:
            try:
                uptime_seconds = float(uptime_output.split()[0])
                hours = int(uptime_seconds // 3600)
                minutes = int((uptime_seconds % 3600) // 60)
                uptime_data['uptime'] = f"{hours}h {minutes}m"
            except:
                uptime_data['uptime'] = uptime_output
        
        if boot_time:
            uptime_data['first_boot'] = boot_time
            
        return uptime_data
    
    def get_running_processes(self) -> List[Dict[str, str]]:
        """Get top running processes"""
        # Try different ps command formats for Android
        ps_output = self.run_adb_command(["shell", "ps"])
        
        if not ps_output:
            # Fallback to simpler ps command
            ps_output = self.run_adb_command(["shell", "ps", "-A"])
        
        processes = []
        lines = ps_output.split('\n')[1:]  # Skip header
        
        for line in lines[:15]:  # Top 15 processes
            if line.strip():
                parts = line.split()
                if len(parts) >= 5:
                    processes.append({
                        'pid': parts[0],
                        'ppid': parts[1],
                        'cpu': parts[2] if parts[2] != '?' else '0',
                        'mem': parts[3] if parts[3] != '?' else '0',
                        'name': ' '.join(parts[4:])
                    })
        
        return processes
    
    def get_installed_apps(self, limit: int = 20) -> List[Dict[str, str]]:
        """Get list of installed apps with more info"""
        # Get user-installed packages
        packages = self.run_adb_command(["shell", "pm", "list", "packages", "-3"])
        apps = []
        
        for package_line in packages.splitlines()[:limit]:
            if package_line.startswith('package:'):
                package_name = package_line.replace('package:', '')
                
                # Try to get app name
                app_info = self.run_adb_command([
                    "shell", "dumpsys", "package", package_name, "|", "grep", "name"
                ])
                
                apps.append({
                    'package': package_name,
                    'info': app_info[:100] if app_info else 'No additional info'
                })
        
        return apps
    
    def get_thermal_info(self) -> Dict[str, Any]:
        """Get thermal information"""
        try:
            thermal_output = self.run_adb_command(["shell", "dumpsys", "thermalservice"])
            thermal_data = {
                "status": "Unknown",
                "temperatures": [],
                "cooling_devices": []
            }

            # Parse thermal status
            status_match = re.search(r"Thermal Status:\s*(\d+)", thermal_output)
            if status_match:
                status_map = {
                    "0": "NONE/NORMAL",
                    "1": "LIGHT",
                    "2": "MODERATE", 
                    "3": "SEVERE",
                    "4": "CRITICAL",
                    "5": "EMERGENCY",
                    "6": "SHUTDOWN",
                }
                thermal_data["status"] = status_map.get(status_match.group(1), status_match.group(1))

            # Parse temperatures
            for line in thermal_output.splitlines():
                temp_match = re.search(r"Temperature\{mValue=([\d.]+).*?mName=([A-Z_]+)", line)
                if temp_match:
                    thermal_data["temperatures"].append({
                        "zone": temp_match.group(2),
                        "temperature": f"{temp_match.group(1)} °C"
                    })

            # Parse cooling devices  
            for line in thermal_output.splitlines():
                cooling_match = re.search(r"CoolingDevice\{mValue=(\d+).*?mName=([A-Za-z0-9_\-]+)", line)
                if cooling_match:
                    thermal_data["cooling_devices"].append({
                        "device": cooling_match.group(2),
                        "level": cooling_match.group(1)
                    })

            return thermal_data
        except Exception:
            return {"status": "unavailable", "temperatures": [], "cooling_devices": []}
    
    def get_bluetooth_info(self) -> Dict[str, Any]:
        """Get Bluetooth information"""
        bt_output = self.run_adb_command(["shell", "dumpsys", "bluetooth_manager"])
        
        bt_data = {
            'enabled': False,
            'devices': [],
            'adapter_name': 'Unknown'
        }
        
        if 'enabled: true' in bt_output.lower():
            bt_data['enabled'] = True
        
        # Parse connected devices
        for line in bt_output.splitlines():
            if 'Device:' in line and 'Connected' in line:
                bt_data['devices'].append(line.strip())
        
        return bt_data
    
    def get_location_info(self) -> Dict[str, str]:
        """Get location services information"""
        location_data = {}
        
        # Location services enabled
        location_mode = self.run_adb_command([
            "shell", "settings", "get", "secure", "location_mode"
        ])
        if location_mode.isdigit():
            modes = {
                "0": "off",
                "1": "device_only",
                "2": "battery_saving", 
                "3": "high_accuracy"
            }
            location_data['mode'] = modes.get(location_mode, location_mode)
        
        # GPS provider
        gps_enabled = self.run_adb_command([
            "shell", "settings", "get", "secure", "location_providers_allowed"
        ])
        if gps_enabled:
            location_data['providers'] = gps_enabled
        
        return location_data
    
    def get_all_device_info(self) -> Dict[str, Any]:
        """Get comprehensive device information"""
        print("Gathering comprehensive device information...")
        start_time = time.time()
        
        device_info = {
            'scan_timestamp': datetime.now().isoformat(),
            'connected_devices': self.get_connected_devices(),
            'basic_info': self.get_basic_device_info(),
            'cpu_info': self.get_cpu_info(),
            'gpu_info': self.get_gpu_info(),
            'memory_info': self.get_memory_info(),
            'storage_info': self.get_storage_info(),
            'battery_info': self.get_battery_info(),
            'network_info': self.get_network_info(),
            'wifi_info': self.get_wifi_info(),
            'display_info': self.get_display_info(),
            'audio_info': self.get_audio_info(),
            'camera_info': self.get_camera_info(),
            'security_info': self.get_security_info(),
            'sensor_info': self.get_sensor_info(),
            'system_uptime': self.get_system_uptime(),
            'running_processes': self.get_running_processes(),
            'installed_apps': self.get_installed_apps(),
            'thermal_info': self.get_thermal_info(),
            'bluetooth_info': self.get_bluetooth_info(),
            'location_info': self.get_location_info()
        }
        
        end_time = time.time()
        device_info['scan_duration'] = f"{end_time - start_time:.2f} seconds"
        
        return device_info
    
    def print_device_info(self, info: Dict[str, Any]):
        """Print device information in a formatted way"""
        print("\n" + "="*70)
        print("           COMPREHENSIVE ANDROID DEVICE INFORMATION")
        print("="*70)
        print(f"Scan completed in: {info.get('scan_duration', 'Unknown')}")
        print(f"Timestamp: {info.get('scan_timestamp', 'Unknown')}")
        
        # Connected Devices
        print(f"\n📱 Connected Devices ({len(info['connected_devices'])}):")
        for device in info['connected_devices']:
            print(f"   • {device['id']} - {device['status']}")
            if 'model' in device:
                print(f"     Model: {device['model']}")
        
        # Basic Info
        basic = info['basic_info']
        print(f"\n🔧 Device Details:")
        print(f"   • Manufacturer: {basic['manufacturer']}")
        print(f"   • Brand: {basic['brand']}")
        print(f"   • Model: {basic['model']}")
        print(f"   • Device: {basic['device']}")
        print(f"   • Android: {basic['android_version']} (API {basic['api_level']})")
        print(f"   • Security Patch: {basic['security_patch']}")
        print(f"   • Build: {basic['build_number']}")
        print(f"   • Fingerprint: {basic['fingerprint'][:60]}...")
        print(f"   • ABI: {basic['abi']}")

        # System Uptime
        uptime = info.get('system_uptime', {})
        if uptime:
            print(f"\n⏱️  System Status:")
            if 'uptime' in uptime:
                print(f"   • Uptime: {uptime['uptime']}")
            if 'first_boot' in uptime:
                print(f"   • First Boot: {uptime['first_boot']}")

        # Display
        disp = info['display_info']
        print(f"\n🖼️  Display:")
        for k, v in disp.items():
            print(f"   • {k.replace('_', ' ').title()}: {v}")
        
        # CPU Info
        cpu = info['cpu_info']
        print(f"\n🖥️  CPU Information:")
        print(f"   • Architecture: {cpu['architecture']}")
        print(f"   • Cores: {cpu['core_count']}")
        if cpu.get('current_usage'):
            print(f"   • Current Usage: {cpu['current_usage']}")
        
        # Show frequency info for each core
        if 'frequencies' in cpu and cpu['frequencies']:
            print(f"   • Core Frequencies:")
            for freq in cpu['frequencies'][:4]:  # Show first 4 cores
                core_info = f"Core {freq['core']}: {freq.get('current_frequency', 'N/A')}"
                if 'max_frequency' in freq:
                    core_info += f" (max: {freq['max_frequency']})"
                print(f"     - {core_info}")
        
        # GPU Info
        gpu = info['gpu_info']
        print(f"\n🎮 GPU Information:")
        print(f"   • Vendor: {gpu['vendor']}")
        print(f"   • Renderer: {gpu['renderer']}")
        print(f"   • Version: {gpu['version']}")
        if gpu.get('extensions'):
            print(f"   • Extensions: {len(gpu['extensions'])} available")
        
        # Memory Info
        memory = info['memory_info']
        print(f"\n💾 Memory Information:")
        for key, value in memory.items():
            if key != 'memory_usage':
                print(f"   • {key.replace('_', ' ').title()}: {value}")
        if 'memory_usage' in memory:
            print(f"   • Usage: {memory['memory_usage']}")
        
        # Storage Information
        print(f"\n💿 Storage Information:")
        storage = info['storage_info']
        if storage.get('total_storage'):
            print(f"   • Total Storage: {storage['total_storage']}")
            print(f"   • Available: {storage['available_storage']}")
        
        important_mounts = ['/data', '/system', '/vendor', '/storage/emulated']
        for partition in storage['partitions']:
            mount = partition['mounted_on']
            if any(mount.startswith(imp) for imp in important_mounts):
                print(f"   • {mount}: {partition['used']}/{partition['size']} ({partition['use_percentage']})")
        
        # Battery Info
        battery = info['battery_info']
        if battery:
            print(f"\n🔋 Battery Information:")
            for key, value in battery.items():
                print(f"   • {key.replace('_', ' ').title()}: {value}")
        
        # Network Info
        network = info['network_info']
        print(f"\n🌐 Network Interfaces:")
        for interface in network.get('interfaces', []):
            if interface.get('addresses'):
                print(f"   • {interface['name']}: {', '.join(interface['addresses'])}")
        
        # WiFi
        wifi = info['wifi_info']
        if wifi:
            print(f"\n📶 WiFi Information:")
            for k, v in wifi.items():
                print(f"   • {k.replace('_', ' ').title()}: {v}")

        # Audio
        audio = info['audio_info']
        print(f"\n🎧 Audio Information:")
        if audio.get('volume_levels'):
            print("   • Volume Levels:")
            for stream, volume in audio['volume_levels'].items():
                print(f"     - {stream.title()}: {volume}")
        if audio.get('audio_devices'):
            print(f"   • Audio Devices: {len(audio['audio_devices'])} found")

        # Camera
        cameras = info['camera_info']
        print(f"\n📷 Cameras ({len(cameras)}):")
        for i, cam in enumerate(cameras[:3]):  # Show first 3 cameras
            print(f"   • Camera {i}: {cam.get('id', 'Unknown')}")
            if cam.get('details'):
                for detail in cam['details'][:2]:  # Show first 2 details
                    print(f"     - {detail}")

        # Security
        security = info['security_info']
        print(f"\n🔐 Security Information:")
        for k, v in security.items():
            print(f"   • {k.replace('_', ' ').title()}: {v}")

        # Thermal
        thermal = info.get('thermal_info', {})
        if thermal and thermal.get('status') != 'unavailable':
            print(f"\n🌡️  Thermal Information:")
            print(f"   • Status: {thermal.get('status', 'Unknown')}")
            
            temps = thermal.get('temperatures', [])
            if temps:
                print("   • Temperatures:")
                for temp in temps[:5]:  # Show first 5 temperature zones
                    print(f"     - {temp.get('zone', 'Unknown')}: {temp.get('temperature', 'N/A')}")
            
            cooling = thermal.get('cooling_devices', [])
            if cooling:
                print(f"   • Cooling Devices: {len(cooling)} active")

        # Bluetooth
        bluetooth = info.get('bluetooth_info', {})
        if bluetooth:
            print(f"\n📘 Bluetooth:")
            print(f"   • Enabled: {bluetooth.get('enabled', False)}")
            if bluetooth.get('devices'):
                print(f"   • Connected Devices: {len(bluetooth['devices'])}")

        # Location
        location = info.get('location_info', {})
        if location:
            print(f"\n📍 Location Services:")
            for k, v in location.items():
                print(f"   • {k.replace('_', ' ').title()}: {v}")

        # Running Processes
        processes = info.get('running_processes', [])
        if processes:
            print(f"\n⚙️  Top Running Processes:")
            for proc in processes[:10]:  # Show top 10
                print(f"   • {proc['name']} (PID: {proc['pid']}) - CPU: {proc['cpu']}%")

        # Sensors
        sensors = info.get('sensor_info', [])
        print(f"\n📡 Sensors ({len(sensors)} found):")
        for sensor in sensors[:10]:  # Show first 10
            if isinstance(sensor, dict):
                print(f"   • {sensor.get('name', 'Unknown')}")
            else:
                print(f"   • {sensor}")
        if len(sensors) > 10:
            print(f"   • ... and {len(sensors) - 10} more")

        # Installed Apps
        apps = info.get('installed_apps', [])
        print(f"\n📦 User Installed Apps ({len(apps)}):")
        for app in apps[:10]:  # Show first 10
            if isinstance(app, dict):
                print(f"   • {app.get('package', 'Unknown')}")
            else:
                print(f"   • {app}")
        if len(apps) > 10:
            print(f"   • ... and {len(apps) - 10} more")

def main():
    """Main function to demonstrate usage"""
    print("Android Device Information Scanner")
    print("==================================")
    
    # Initialize ADB Device Info reader
    adb_info = ADBDeviceInfo()
    
    # Check for connected devices first
    devices = adb_info.get_connected_devices()
    
    if not devices:
        print("❌ No devices connected. Please:")
        print("   1. Connect an Android device via USB")
        print("   2. Enable Developer Options")
        print("   3. Enable USB Debugging")
        print("   4. Accept the debugging prompt on your device")
        return
    
    if len(devices) == 1:
        device_id = devices[0]['id']
        adb_info = ADBDeviceInfo(device_id)
        print(f"✅ Using device: {device_id}")
    else:
        print(f"📱 Multiple devices found ({len(devices)}):")
        for i, device in enumerate(devices):
            print(f"   {i+1}. {device['id']} - {device['status']}")
        
        try:
            choice = input(f"\nSelect device (1-{len(devices)}) or press Enter for first: ").strip()
            if choice and choice.isdigit():
                idx = int(choice) - 1
                if 0 <= idx < len(devices):
                    device_id = devices[idx]['id']
                else:
                    device_id = devices[0]['id']
            else:
                device_id = devices[0]['id']
        except:
            device_id = devices[0]['id']
        
        adb_info = ADBDeviceInfo(device_id)
        print(f"✅ Using device: {device_id}")
    
    # Get all device information
    try:
        device_info = adb_info.get_all_device_info()
        adb_info.print_device_info(device_info)
        
        # Optionally save to JSON file
        print(f"\n{'='*70}")
        save_json = input("💾 Save detailed info to JSON file? (y/n): ").lower().strip()
        if save_json == 'y':
            filename = f"device_info_{device_id.replace(':', '_')}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            try:
                with open(filename, 'w') as f:
                    json.dump(device_info, f, indent=2)
                print(f"✅ Device information saved to {filename}")
            except Exception as e:
                print(f"❌ Error saving file: {e}")
            
    except KeyboardInterrupt:
        print("\n⏹️  Operation cancelled by user.")
    except Exception as e:
        print(f"❌ Error gathering device information: {e}")
        print("   • Make sure ADB is installed and in your PATH")
        print("   • Check that USB debugging is enabled")
        print("   • Verify the device is properly connected")


if __name__ == "__main__":
    main()
