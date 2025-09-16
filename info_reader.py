#!/usr/bin/env python3
"""
Android Device Information Reader using ADB
Reads CPU, GPU, RAM, device info and more from Android devices via ADB
"""

import subprocess
import json
import re
from typing import Dict, List, Optional, Any

class ADBDeviceInfo:
    def __init__(self, device_id: Optional[str] = None):
        """
        Initialize ADB Device Info reader
        
        Args:
            device_id: Specific device ID to target (optional)
        """
        self.device_id = device_id
        self.device_cmd = ["-s", device_id] if device_id else []
    
    def run_adb_command(self, command: List[str]) -> str:
        """Execute ADB command and return output"""
        try:
            full_command = ["adb"] + self.device_cmd + command
            result = subprocess.run(
                full_command, 
                capture_output=True, 
                text=True, 
                timeout=30
            )
            if result.returncode != 0:
                print(f"ADB command failed: {' '.join(full_command)}")
                print(f"Error: {result.stderr}")
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
            'serial': props.get('ro.serialno', 'Unknown')
        }
    
    def get_cpu_info(self) -> Dict[str, Any]:
        """Get CPU information"""
        # Get CPU info from /proc/cpuinfo
        cpuinfo = self.run_adb_command(["shell", "cat", "/proc/cpuinfo"])
        
        cpu_data = {
            'cores': [],
            'architecture': 'Unknown',
            'features': []
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
        
        if current_core:
            cpu_data['cores'].append(current_core)
        
        cpu_data['core_count'] = core_count
        
        # Get CPU frequencies
        freq_info = self.get_cpu_frequencies()
        cpu_data.update(freq_info)
        
        return cpu_data
    
    def get_cpu_frequencies(self) -> Dict[str, Any]:
        """Get CPU frequency information"""
        # Try to get scaling frequencies
        max_freq = self.run_adb_command([
            "shell", 
            "cat", 
            "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq"
        ])
        
        min_freq = self.run_adb_command([
            "shell", 
            "cat", 
            "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_min_freq"
        ])
        
        current_freq = self.run_adb_command([
            "shell", 
            "cat", 
            "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"
        ])
        
        freq_info = {}
        
        if max_freq and max_freq.isdigit():
            freq_info['max_frequency'] = f"{int(max_freq) // 1000} MHz"
        
        if min_freq and min_freq.isdigit():
            freq_info['min_frequency'] = f"{int(min_freq) // 1000} MHz"
        
        if current_freq and current_freq.isdigit():
            freq_info['current_frequency'] = f"{int(current_freq) // 1000} MHz"
        
        return freq_info
    
    def get_gpu_info(self) -> Dict[str, Any]:
        """Get GPU information"""
        # Try different methods to get GPU info
        gpu_info = {
            'renderer': 'Unknown',
            'vendor': 'Unknown',
            'version': 'Unknown'
        }
        
        # Method 1: Try dumpsys SurfaceFlinger
        dumpsys_output = self.run_adb_command(["shell", "dumpsys", "SurfaceFlinger"])
        
        for line in dumpsys_output.split('\n'):
            if 'GLES:' in line:
                # Extract GL ES info
                gl_info = line.split('GLES:')[1].strip()
                parts = gl_info.split(',')
                if len(parts) >= 3:
                    gpu_info['vendor'] = parts[0].strip()
                    gpu_info['renderer'] = parts[1].strip()
                    gpu_info['version'] = parts[2].strip()
                break
        
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
                
                if key in ['MemTotal', 'MemFree', 'MemAvailable', 'Buffers', 'Cached']:
                    # Convert kB to MB
                    if 'kB' in value:
                        kb_value = int(value.split()[0])
                        memory_data[key.lower()] = f"{kb_value // 1024} MB"
                    else:
                        memory_data[key.lower()] = value
        
        # Get additional memory info from dumpsys meminfo
        dumpsys_mem = self.run_adb_command(["shell", "dumpsys", "meminfo"])
        
        for line in dumpsys_mem.split('\n')[:10]:  # Check first few lines
            if 'Total RAM:' in line:
                memory_data['total_ram'] = line.split('Total RAM:')[1].strip()
                break
        
        return memory_data
    
    def get_storage_info(self) -> Dict[str, Any]:
        """Get storage information"""
        # Get disk usage info
        df_output = self.run_adb_command(["shell", "df", "-h"])
        
        storage_data = {
            'partitions': []
        }
        
        for line in df_output.split('\n')[1:]:  # Skip header
            if line.strip():
                parts = line.split()
                if len(parts) >= 6:
                    partition = {
                        'filesystem': parts[0],
                        'size': parts[1],
                        'used': parts[2],
                        'available': parts[3],
                        'use_percentage': parts[4],
                        'mounted_on': parts[5]
                    }
                    storage_data['partitions'].append(partition)
        
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
        
        return battery_data
    
    def get_network_info(self) -> Dict[str, Any]:
        """Get network interface information"""
        # Get network interfaces
        ifconfig = self.run_adb_command(["shell", "ip", "addr"])
        
        if not ifconfig:  # Fallback to ifconfig if ip command fails
            ifconfig = self.run_adb_command(["shell", "ifconfig"])
        
        # Get WiFi info
        wifi_info = self.run_adb_command(["shell", "dumpsys", "wifi"])
        
        network_data = {
            'interfaces': ifconfig[:500] if ifconfig else "Unable to retrieve",  # Truncate for readability
            'wifi_connected': 'enabled' in wifi_info.lower() if wifi_info else False
        }
        
        return network_data
    
    def get_sensor_info(self) -> List[str]:
        """Get available sensors"""
        sensor_output = self.run_adb_command(["shell", "dumpsys", "sensorservice"])
        
        sensors = []
        
        for line in sensor_output.split('\n'):
            if 'Sensor[' in line:
                sensor_match = re.search(r'Sensor\[.*?\]\s+(.+)', line)
                if sensor_match:
                    sensor_name = sensor_match.group(1).strip()
                    if sensor_name not in sensors:
                        sensors.append(sensor_name)
        
        return sensors[:20]  # Limit to first 20 sensors for readability
    
        # ---------------- NEW EXTRA METHODS ---------------- #

    def get_display_info(self) -> Dict[str, str]:
        """Get screen resolution, density, refresh rate, brightness"""
        display_data = {}
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
            display_data["brightness"] = brightness

        # Brightness mode (0 = manual, 1 = auto)
        mode = self.run_adb_command(
            ["shell", "settings", "get", "system", "screen_brightness_mode"]
        )
        if mode in ["0", "1"]:
            display_data["brightness_mode"] = "auto" if mode == "1" else "manual"

        # Refresh rate (SurfaceFlinger)
        sf = self.run_adb_command(["shell", "dumpsys", "SurfaceFlinger"])
        match = re.search(r"(\d+(\.\d+)?) fps", sf)
        if match:
            display_data["refresh_rate"] = f"{match.group(1)} fps"

        return display_data

    def get_wifi_info(self) -> Dict[str, str]:
        """Get WiFi SSID, signal strength, link speed"""
        wifi_output = self.run_adb_command(["shell", "dumpsys", "wifi"])
        wifi_data = {}
        for line in wifi_output.splitlines():
            line = line.strip()
            if "SSID:" in line and "real" not in wifi_data:
                wifi_data["ssid"] = line.split("SSID:")[-1].strip()
            if "RSSI:" in line:
                wifi_data["signal_strength"] = line.split("RSSI:")[-1].strip()
            if "Link speed" in line:
                wifi_data["link_speed"] = line.split("Link speed:")[-1].strip()
        return wifi_data

    def get_audio_info(self) -> Dict[str, str]:
        """Get volume levels and connected audio devices"""
        audio_output = self.run_adb_command(["shell", "dumpsys", "audio"])
        audio_data = {}
        for line in audio_output.splitlines():
            if "STREAM_MUSIC:" in line:
                audio_data["music_stream"] = line.strip()
            if "Devices:" in line and "connected" not in audio_data:
                audio_data["devices"] = line.strip()
        return audio_data

    def get_camera_info(self) -> List[str]:
        """Get available cameras and their characteristics"""
        cam_output = self.run_adb_command(["shell", "dumpsys", "media.camera"])
        cameras = []
        for line in cam_output.splitlines():
            if "Camera ID" in line:
                cameras.append(line.strip())
        return cameras

    def get_security_info(self) -> Dict[str, str]:
        """Get encryption, SELinux, security state"""
        security_data = {}
        crypto = self.run_adb_command(["shell", "getprop", "ro.crypto.state"])
        if crypto:
            security_data["encryption"] = crypto
        selinux = self.run_adb_command(["shell", "getenforce"])
        if selinux:
            security_data["selinux"] = selinux
        return security_data

    def get_installed_apps(self) -> List[str]:
        """Get list of installed apps (package names)"""
        packages = self.run_adb_command(["shell", "pm", "list", "packages", "-3"])
        return [p.replace("package:", "") for p in packages.splitlines() if p]
    
    # Thermal Info
    def get_thermal_info(self):
        try:
            out = self.run_adb_command(["shell", "dumpsys", "thermalservice"])
            info = {"status": None, "temps": [], "cooling": []}

            # Parse thermal status
            match = re.search(r"Thermal Status:\s*(\d+)", out)
            if match:
                status_map = {
                    "0": "NONE/NORMAL",
                    "1": "LIGHT",
                    "2": "MODERATE",
                    "3": "SEVERE",
                    "4": "CRITICAL",
                    "5": "EMERGENCY",
                    "6": "SHUTDOWN",
                }
                info["status"] = status_map.get(match.group(1), match.group(1))

            # Parse current temps
            for line in out.splitlines():
                m = re.search(r"Temperature\{mValue=([\d.]+).*?mName=([A-Z_]+)", line)
                if m:
                    info["temps"].append(f"{m.group(2)}: {m.group(1)} °C")

            # Parse cooling devices
            for line in out.splitlines():
                m = re.search(r"CoolingDevice\{mValue=(\d+).*?mName=([A-Za-z0-9_\-]+)", line)
                if m:
                    info["cooling"].append(f"{m.group(2)} (value={m.group(1)})")

            return info
        except Exception:
            return {"status": "unavailable", "temps": [], "cooling": []}

    
    def get_all_device_info(self) -> Dict[str, Any]:
        """Get comprehensive device information"""
        print("Gathering device information...")
        
        device_info = {
            'connected_devices': self.get_connected_devices(),
            'basic_info': self.get_basic_device_info(),
            'cpu_info': self.get_cpu_info(),
            'gpu_info': self.get_gpu_info(),
            'memory_info': self.get_memory_info(),
            'storage_info': self.get_storage_info(),
            'battery_info': self.get_battery_info(),
            'network_info': self.get_network_info(),
            'sensors': self.get_sensor_info(),
            'wifi_info': self.get_wifi_info(),
            'display_info': self.get_display_info(),
            'audio_info': self.get_audio_info(),
            'camera_info': self.get_camera_info(),
            'security_info': self.get_security_info(),
            'installed_apps': self.get_installed_apps()[:20],  # limit for readability
            'thermal': self.get_thermal_info(),
        }
        
        return device_info
    
    def print_device_info(self, info: Dict[str, Any]):
        """Print device information in a formatted way"""
        print("\n" + "="*60)
        print("           ANDROID DEVICE INFORMATION")
        print("="*60)
        
        # Connected Devices
        print(f"\n📱 Connected Devices ({len(info['connected_devices'])}):")
        for device in info['connected_devices']:
            print(f"   • {device['id']} - {device['status']}")
        
        # Basic Info
        basic = info['basic_info']
        print(f"\n🔧 Device Details:")
        print(f"   • Manufacturer: {basic['manufacturer']}")
        print(f"   • Brand: {basic['brand']}")
        print(f"   • Model: {basic['model']}")
        print(f"   • Android: {basic['android_version']} (API {basic['api_level']})")
        print(f"   • Security Patch: {basic['security_patch']}")

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
        if cpu.get('max_frequency'):
            print(f"   • Max Frequency: {cpu['max_frequency']}")
        if cpu.get('current_frequency'):
            print(f"   • Current Frequency: {cpu['current_frequency']}")
        
        # GPU Info
        gpu = info['gpu_info']
        print(f"\n🎮 GPU Information:")
        print(f"   • Vendor: {gpu['vendor']}")
        print(f"   • Renderer: {gpu['renderer']}")
        print(f"   • Version: {gpu['version']}")
        
        # Memory Info
        memory = info['memory_info']
        print(f"\n💾 Memory Information:")
        for key, value in memory.items():
            print(f"   • {key.replace('_', ' ').title()}: {value}")
        
        # Storage Info
        storage = info['storage_info']
        print(f"\n💿 Storage Information:")
        for partition in storage['partitions'][:5]:  # Show first 5 partitions
            if '/data' in partition['mounted_on'] or '/system' in partition['mounted_on']:
                print(f"   • {partition['mounted_on']}: {partition['used']}/{partition['size']} ({partition['use_percentage']})")
        
        # Battery Info
        battery = info['battery_info']
        if battery:
            print(f"\n🔋 Battery Information:")
            for key, value in battery.items():
                print(f"   • {key.replace('_', ' ').title()}: {value}")
        
        # Sensors
        sensors = info['sensors']
        if sensors:
            print(f"\n📡 Available Sensors ({len(sensors)}):")
            for sensor in sensors[:10]:  # Show first 10
                print(f"   • {sensor}")
            if len(sensors) > 10:
                print(f"   • ... and {len(sensors) - 10} more")

        # WiFi
        print(f"\n📶 WiFi:")
        for k, v in info['wifi_info'].items():
            print(f"   • {k}: {v}")

        # Audio
        print(f"\n🎧 Audio:")
        for k, v in info['audio_info'].items():
            print(f"   • {k}: {v}")

        # Camera
        print(f"\n📷 Cameras ({len(info['camera_info'])}):")
        for cam in info['camera_info']:
            print(f"   • {cam}")

        # Security
        print(f"\n🔐 Security:")
        for k, v in info['security_info'].items():
            print(f"   • {k}: {v}")

        # Sensors
        print(f"\n📡 Sensors: {len(info['sensors'])} found")
        for s in info['sensors'][:10]:
            print(f"   • {s}")

        # Installed Apps
        print(f"\n📦 Installed Apps (first 20):")
        for app in info['installed_apps']:
            print(f"   • {app}")

        # Thermal
        if "thermal" in info:
            print("\n🌡️ Thermal:")
            thermal = info["thermal"]

            # status
            status = thermal.get("status", "None")
            print(f"   • status: {status}")

            # temperatures
            temps = thermal.get("temps", [])
            if temps:
                print("   • temps:")
                for t in temps:
                    if isinstance(t, dict):
                        print(f"      - {t.get('zone')}: {t.get('temp')}")
                    else:
                        print(f"      - {t}")
            else:
                print("   • temps: None")

            # cooling devices
            cooling = thermal.get("cooling_devices", [])
            if cooling:
                print("   • cooling devices:")
                for c in cooling:
                    print(f"      - {c}")
            else:
                print("   • cooling devices: None")

def main():
    """Main function to demonstrate usage"""
    # Initialize ADB Device Info reader
    adb_info = ADBDeviceInfo()
    
    # Check for connected devices first
    devices = adb_info.get_connected_devices()
    
    if not devices:
        print("No devices connected. Please connect an Android device with USB debugging enabled.")
        return
    
    if len(devices) == 1:
        # Use the only connected device
        device_id = devices[0]['id']
        adb_info = ADBDeviceInfo(device_id)
        print(f"Using device: {device_id}")
    else:
        # Multiple devices - let user choose or use first one
        print(f"Multiple devices found ({len(devices)}). Using first device: {devices[0]['id']}")
        device_id = devices[0]['id']
        adb_info = ADBDeviceInfo(device_id)
    
    # Get all device information
    try:
        device_info = adb_info.get_all_device_info()
        adb_info.print_device_info(device_info)
        
        # Optionally save to JSON file
        save_json = input("\nSave detailed info to JSON file? (y/n): ").lower().strip()
        if save_json == 'y':
            filename = f"device_info_{device_id.replace(':', '_')}.json"
            with open(filename, 'w') as f:
                json.dump(device_info, f, indent=2)
            print(f"Device information saved to {filename}")
            
    except KeyboardInterrupt:
        print("\nOperation cancelled by user.")
    except Exception as e:
        print(f"Error gathering device information: {e}")


if __name__ == "__main__":
    main()
