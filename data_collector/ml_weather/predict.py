#!/usr/bin/env python3

import pandas as pd
import numpy as np
import joblib
import os
from datetime import datetime
import json
import socket
import sys
import time

class WeatherPredictor:
    def __init__(self, model_dir: str = "models"):
        """
        Load the trained model and scaler
        """
        self.model = joblib.load(os.path.join(model_dir, 'weather_model.joblib'))
        self.scaler = joblib.load(os.path.join(model_dir, 'scaler.joblib'))
    
    def prepare_drone_data(self, drone_data: dict) -> pd.DataFrame:
        """
        Prepare drone data for prediction
        """
        # Extract current hour and month
        current_time = datetime.utcnow()
        
        # Create feature dictionary
        features = {
            'hour': current_time.hour,
            'month': current_time.month,
            'relative_humidity_2m': drone_data.get('humidity', 0),
            'pressure_msl': drone_data.get('pressure', 0),
            'wind_speed_10m': np.sqrt(
                drone_data.get('twist.linear.x', 0)**2 + 
                drone_data.get('twist.linear.y', 0)**2
            ),
            'wind_direction_10m': np.degrees(
                np.arctan2(
                    drone_data.get('twist.linear.y', 0),
                    drone_data.get('twist.linear.x', 0)
                )
            ) % 360,
            'cloud_cover': 0  # This might not be available from drone data
        }
        
        return pd.DataFrame([features])
    
    def predict(self, features: pd.DataFrame) -> float:
        """
        Make temperature prediction
        """
        X_scaled = self.scaler.transform(features)
        prediction = self.model.predict(X_scaled)[0]
        return prediction

def process_drone_data(data: dict) -> dict:
    """
    Process incoming drone data and make weather prediction
    """
    try:
        predictor = WeatherPredictor()
        
        # Prepare features from drone data
        features = predictor.prepare_drone_data(data)
        
        # Make prediction
        predicted_temp = predictor.predict(features)
        
        return {
            'timestamp': datetime.utcnow().isoformat(),
            'predicted_temperature': round(predicted_temp, 2),
            'confidence': 0.95  # This is a placeholder
        }
        
    except Exception as e:
        print(f"Error making prediction: {str(e)}")
        return None

def run_tcp_client(host='127.0.0.1', port=9000):
    predictor = WeatherPredictor()
    print(f"Connecting to TCPLogServer at {host}:{port}...")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((host, port))
            print("Connected. Waiting for data...")
            buffer = ''
            last_prediction_time = 0
            latest_drone_data = None
            while True:
                data = s.recv(4096)
                if not data:
                    print("Connection closed by server.")
                    break
                buffer += data.decode('utf-8')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    try:
                        drone_data = json.loads(line)
                        latest_drone_data = drone_data
                        now = time.time()
                        # Only predict once per minute
                        if now - last_prediction_time >= 60:
                            features = predictor.prepare_drone_data(latest_drone_data)
                            predicted_temp = predictor.predict(features)
                            print(json.dumps({
                                'timestamp': datetime.utcnow().isoformat(),
                                'predicted_temperature': round(predicted_temp, 2),
                                'input': latest_drone_data
                            }, indent=2))
                            last_prediction_time = now
                    except Exception as e:
                        print(f"Error processing data: {e}")
        except Exception as e:
            print(f"Could not connect to server: {e}")
            sys.exit(1)

def predict_from_csv(csv_path):
    import pandas as pd
    predictor = WeatherPredictor()
    print(f"Loading CSV: {csv_path}")
    df = pd.read_csv(csv_path)
    # Find the last row with at least one non-null value in the relevant columns
    for idx in range(len(df)-1, -1, -1):
        row = df.iloc[idx]
        if not (pd.isnull(row.get('twist.linear.x', None)) and pd.isnull(row.get('twist.linear.y', None))):
            drone_data = {
                'humidity': row.get('relative_humidity_2m', 0),
                'pressure': row.get('pressure_msl', 0),
                'twist.linear.x': row.get('twist.linear.x', 0),
                'twist.linear.y': row.get('twist.linear.y', 0),
            }
            features = predictor.prepare_drone_data(drone_data)
            predicted_temp = predictor.predict(features)
            print(json.dumps({
                'timestamp': row.get('timestamp', ''),
                'predicted_temperature': round(predicted_temp, 2),
                'input': drone_data
            }, indent=2))
            break

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Weather prediction TCP client or CSV batch mode.")
    parser.add_argument('--tcp', action='store_true', help='Run as TCP client to receive data from DroneLogger')
    parser.add_argument('--csv', type=str, help='Path to drone log CSV file for batch prediction')
    args = parser.parse_args()
    if args.tcp:
        run_tcp_client()
    elif args.csv:
        predict_from_csv(args.csv)
    else:
        # Example usage with sample drone data
        sample_data = {
            'humidity': 65.0,
            'pressure': 1013.25,
            'twist.linear.x': 2.5,
            'twist.linear.y': 1.5,
        }
        
        result = process_drone_data(sample_data)
        if result:
            print(json.dumps(result, indent=2))
