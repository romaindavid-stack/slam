import sys
import pandas as pd
from pathlib import Path

# --- ROS2 IMPORTE (Die eingebauten Werkzeuge nutzen) ---
try:
    from rclpy.serialization import deserialize_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    
    # Nachrichtentypen importieren (Standard ROS Typen)
    from std_msgs.msg import Float64
    from nav_msgs.msg import Odometry
except ImportError:
    print("\n❌ FEHLER: ROS2 Bibliotheken nicht gefunden!")
    print("   Hast du 'source /opt/ros/humble/setup.bash' ausgeführt?")
    sys.exit(1)

# ==========================================
# HIER EINSTELLUNGEN ÄNDERN
# ==========================================

# 1. Wo liegt dein Rosbag? (Ordnername oder Pfad)
BAG_PATH = "/home/corrosionfly/ros2_lio_ws/messfahrt_01" 

# 2. Wie heißen die Topics?
TOPIC_VOLT = '/keithley/measurement'   # Das Voltmeter (Typ: Float64)
TOPIC_POS  = '/Odometry'               # Die Position (Typ: Odometry)

# ==========================================

def get_rosbag_data(bag_path):
    reader = SequentialReader()
    
    # Optionen setzen (Standard ROS2 Bag Format)
    storage_options = StorageOptions(uri=bag_path, storage_id='mcap') # Oder 'sqlite3' falls mcap nicht geht
    converter_options = ConverterOptions('', '')
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Konnte Bag nicht öffnen (Versuche sqlite3...): {e}")
        # Fallback auf sqlite3 (das alte Format)
        storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        try:
            reader.open(storage_options, converter_options)
        except Exception as e2:
            print(f"❌ Kritischer Fehler beim Öffnen: {e2}")
            return None, None

    keithley_data = []
    odom_data = []
    
    print(f"Lese Bag: {bag_path} ...")

    while reader.has_next():
        (topic, data, t_ns) = reader.read_next()
        t_sec = t_ns * 1e-9

        if topic == TOPIC_VOLT:
            msg = deserialize_message(data, Float64)
            keithley_data.append({'time': t_sec, 'voltage': msg.data})
            
        elif topic == TOPIC_POS:
            msg = deserialize_message(data, Odometry)
            pos = msg.pose.pose.position
            odom_data.append({
                'time': t_sec,
                'x': pos.x, 'y': pos.y, 'z': pos.z
            })

    return pd.DataFrame(keithley_data), pd.DataFrame(odom_data)

def run_processing():
    bag_path_obj = Path(BAG_PATH)
    
    # 1. Daten einlesen
    df_volt, df_odom = get_rosbag_data(BAG_PATH)
    
    if df_volt is None or df_volt.empty:
        print(f"❌ Keine Spannungsdaten auf {TOPIC_VOLT} gefunden.")
        return
    if df_odom is None or df_odom.empty:
        print(f"❌ Keine Positionsdaten auf {TOPIC_POS} gefunden.")
        return

    # 2. Interpolieren (Synchronisieren)
    print(f"Verbinde {len(df_volt)} Messwerte mit Positionen...")
    
    df_odom = df_odom.sort_values('time').reset_index(drop=True)
    results = []

    for _, row in df_volt.iterrows():
        t_target = row['time']
        
        # Finde passenden Zeitindex
        idx = df_odom['time'].searchsorted(t_target)

        if idx == 0 or idx >= len(df_odom): continue
        
        prev = df_odom.iloc[idx - 1]
        next_ = df_odom.iloc[idx]

        # Wenn Lücke zu groß (>0.5s), überspringen
        if (next_['time'] - prev['time']) > 0.5: continue

        # Lineare Interpolation
        alpha = (t_target - prev['time']) / (next_['time'] - prev['time'])

        results.append({
            'Timestamp': t_target,
            'Voltage': row['voltage'],
            'X': prev['x'] + alpha * (next_['x'] - prev['x']),
            'Y': prev['y'] + alpha * (next_['y'] - prev['y']),
            'Z': prev['z'] + alpha * (next_['z'] - prev['z'])
        })

    # 3. Speichern
    final_df = pd.DataFrame(results)
    output_file = bag_path_obj.with_name(f"{bag_path_obj.name}_fertig.csv")
    final_df.to_csv(output_file, index=False)
    
    print(f"\n✅ ERFOLG! Datei gespeichert:")
    print(f"   {output_file}")

if __name__ == '__main__':
    run_processing()