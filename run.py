import subprocess
import sys
import time

def run_make_command():
    try:
        print("Waiting for Docker to initialize...")
        time.sleep(5)  # Wait 10 seconds for Docker to start

        # Start the process
        process = subprocess.Popen(['make', 'run_all'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        # Read output line by line
        for line in process.stdout:
            print(line, end='')  # Print each line as it is produced

        # Wait for the process to complete and get the return code
        return_code = process.wait()

        # Check for errors
        if return_code != 0:
            error_output = process.stderr.read()
            print(f"An error occurred: {error_output}")
    except KeyboardInterrupt:
        print("\nProcess interrupted. Stopping...")
        process.terminate()  # Terminate the subprocess
        sys.exit(0)  # Exit the script
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    run_make_command()

