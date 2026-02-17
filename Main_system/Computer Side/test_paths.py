import os
from pathlib import Path

def test_env_resolution():
    print("Testing .env path resolution...")
    
    # Simulate being in MQTT8/main.py
    current_file = Path("MQTT8/main.py").resolve()
    env_path = current_file.parent.parent / '.env'
    
    print(f"Simulated location: {current_file}")
    print(f"Calculated .env path: {env_path}")
    
    if env_path.exists():
        print("✅ SUCCESS: .env file found at calculated path!")
        content = env_path.read_text()
        if "MODEL=gpt-4o-mini" in content:
             print("✅ Content verified: found MODEL setting.")
        else:
             print("⚠️ Warning: .env found but content looks different.")
    else:
        print("❌ FAILURE: .env file NOT found at calculated path.")
        print(f"Directory listing of {env_path.parent}:")
        try:
            for f in env_path.parent.iterdir():
                print(f" - {f.name}")
        except Exception as e:
            print(f"Error listing dir: {e}")

if __name__ == "__main__":
    test_env_resolution()
