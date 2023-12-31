import tkinter as tk
import tkinter.font as tkFont
from tkinter import ttk
from tkinter import Label, Button, Entry, StringVar, Frame, OptionMenu, Checkbutton, IntVar, messagebox
from smbus2 import SMBus
from threading import Thread
import time




class MotorController:
    def __init__(self):
        
        self.dev_addr = 0x29 # adress if switch set to on. 0x28 if set to 1
        self.bus = SMBus(0) # I2C SDA(27), SCL(28) AGX bus1, NANO bus0

    
    def miranda_write(self, address, command, tx_data):
        try:
            self.bus.write_i2c_block_data(address, command, tx_data)
        except Exception as e:
            print(f"Failed to write to Miranda: {e}")

    def miranda_read(self, address, command, num_bytes):
        try:
            return self.bus.read_i2c_block_data(address, command, num_bytes)
        except Exception as e:
            print(f"Failed to read from Miranda: {e}")
            return None


class MotorTuner:
    def __init__(self, master, motor_controller):
        self.master = master
        self.master.title("Miranda Motor Control")
        self.motor_controller = motor_controller
        self.prev_encoder_pos = None
        self.motor_running = False
        self.text_var = tk.StringVar()
        self.text_var.set("Motor stopped")
        self.create_motor_status_label()
        self.create_encoder_entry()
        self.dtm_output = None
        self.create_dtm_output_box()
        self.create_toggle_dtm_button()
        self.create_angle_entry()
        self.create_manual_angle_entry()
        self.create_accel_entry()
        self.create_accel_setting_entry()
        self.create_pid_gain_entries()
        self.create_rpm_control()
        self.create_home_motor_button()
        self.create_velocity_motion_label()
        self.create_absolute_position_label()
        self.create_abs_at_rpm_button()
        self.create_time_entry()
        self.create_abs_in_seconds_button()
        self.create_relative_position_label()
        self.create_move_relative_at_speed_button()
        self.create_move_relative_in_time_button()
        self.create_control_buttons()
        self.encoder_value = StringVar()
        self.update_encoder_display()
        self.create_flash_pid_button()  # Create the FLASH PID Gains button
        self.conversion_factor = 91.01944444444445 # 1 degree/second ~ 91 motor counts/second
        self.counts_per_rpm = self.conversion_factor * 60
        self.create_reload_defaults_button()
        self.create_exit_button()
        

        current_acceleration = self.get_acceleration()
        if current_acceleration is not None:
            self.accel_entry.insert(0, str(current_acceleration))
            
        current_dtm = self.get_dtm_status()
        if current_dtm is not None:
            self.dtm_output.insert(0, str(current_dtm))    
    


    # Add label to show motor status
    def create_motor_status_label(self):
        self.motor_status_label = ttk.Label(self.master, textvariable=self.text_var)
        self.motor_status_label.grid(row=1, column=3, columnspan=2, padx=10, pady=20, sticky='ew')
        
    def create_angle_entry(self):
        ttk.Label(self.master, text="Angle:").grid(row=2, column=0, pady=2, sticky='e')
        self.angle_entry = ttk.Entry(self.master, width=6, justify="center")
        self.angle_entry.grid(row=2, column=1, pady=10)
        
    def create_accel_entry(self):
        ttk.Label(self.master, text="Acceleration:").grid(row=2, column=3, pady=2, sticky='e')
        self.accel_entry = ttk.Entry(self.master, width=6, justify="center")
        self.accel_entry.grid(row=2, column=4, pady=10)

    def create_encoder_entry(self):
        ttk.Label(self.master, text="Encoder:").grid(row=2, column=6, pady=2, sticky='e')
        self.encoder_entry = ttk.Entry(self.master, width=6, justify="center")
        self.encoder_entry.grid(row=2, column=7, padx=10, pady=10)
   

    # Add controls
    def create_toggle_dtm_button(self):
        toggle_dtm_button = ttk.Button(self.master, text="Toggle DTM", command=self.toggle_dtm)
        toggle_dtm_button.grid(row=3, column=0, padx=10, pady=10, sticky='e')
    
    def create_dtm_output_box(self):
        self.dtm_output = ttk.Entry(self.master, width=6, justify="center", state="readonly")
        self.dtm_output.grid(row=3, column=1, padx=10, pady=10)   
   
    def create_pid_gain_entries(self):
        ttk.Label(self.master, text="P Gain:").grid(row=4, column=0, pady=2, sticky='e')
        self.kp_entry = ttk.Entry(self.master, width=6, justify="center")
        self.kp_entry.grid(row=4, column=1, pady=2)

        ttk.Label(self.master, text="I Gain:").grid(row=4, column=2, pady=2)
        self.ki_entry = ttk.Entry(self.master, width=6, justify="center")
        self.ki_entry.grid(row=4, column=3, pady=2)

        ttk.Label(self.master, text="D Gain:").grid(row=4, column=4, pady=2)
        self.kd_entry = ttk.Entry(self.master, width=6, justify="center")
        self.kd_entry.grid(row=4, column=5, pady=2)

        ttk.Label(self.master, text="Kc Gain:").grid(row=4, column=6, pady=2)
        self.kc_entry = ttk.Entry(self.master, width=6, justify="center")
        self.kc_entry.grid(row=4, column=7, padx=10, pady=2)

    def create_accel_setting_entry(self):
        set_accel_button = ttk.Button(self.master, text="Set Acceleration", command=self.set_acceleration)
        set_accel_button.grid(row=8, column=0, padx=10, pady=10, sticky='e')
        self.accel_setting_entry = ttk.Entry(self.master, width=6, justify="center")
        self.accel_setting_entry.grid(row=8, column=1, pady=2, sticky='w')
        
    def create_rpm_control(self):
        ttk.Label(self.master, text="RPM:").grid(row=8, column=2, pady=2, sticky='e')
        self.rpm_entry = ttk.Entry(self.master, width=6, justify="center")
        self.rpm_entry.grid(row=8, column=3, pady=2)
       
    def create_manual_angle_entry(self):
        ttk.Label(self.master, text="Angle:").grid(row=8, column=4, pady=2, sticky='e')
        self.manual_angle_entry = ttk.Entry(self.master, width=6, justify="center")
        self.manual_angle_entry.grid(row=8, column=5, pady=2)
        
    def create_time_entry(self):
        ttk.Label(self.master, text="Time (sec):").grid(row=8, column=6, pady=2, sticky='e')
        self.time_entry = ttk.Entry(self.master, width=6, justify="center")
        self.time_entry.grid(row=8, column=7, pady=2)
        
    def create_home_motor_button(self):
        home_motor_button = ttk.Button(self.master, text="Home Motor", command=self.home_motor)
        home_motor_button.grid(row=9, column=3, columnspan=2, padx=10, pady=10, sticky='ew')
        
    def create_velocity_motion_label(self):
        velocity_motion_label = ttk.Label(self.master, text="Velocity Motion")
        velocity_motion_label.grid(row=10, column=3, columnspan=2, padx=10, pady=10, sticky='ew')
        
    def create_control_buttons(self):
        start_button = ttk.Button(self.master, text="Start Motor", command=self.start_motor)
        start_button.grid(row=10, column=1, padx=10, pady=10)

        stop_button = ttk.Button(self.master, text="Stop Motor", command=self.stop_motor)
        stop_button.grid(row=10, column=6, padx=10, pady=10)
        
    def create_absolute_position_label(self):
        absolute_position_label = ttk.Label(self.master, text="Absolute Position")
        absolute_position_label.grid(row=12, column=3, columnspan=2, padx=10, pady=10, sticky='ew')       
        
    def create_abs_at_rpm_button(self):
        abs_at_rpm_button = ttk.Button(self.master, text="ABS at RPM", command=self.move_abs_at_rpm)
        abs_at_rpm_button.grid(row=12, column=1, padx=10, pady=10)   

    def create_abs_in_seconds_button(self):
        abs_in_seconds_button = ttk.Button(self.master, text="ABS Pos in Time", command=self.move_abs_in_set_time)
        abs_in_seconds_button.grid(row=12, column=6, padx=10, pady=10)
        
    def create_relative_position_label(self):
        relative_position_label = ttk.Label(self.master, text="Relative Position")
        relative_position_label.grid(row=14, column=3, columnspan=2, padx=10, pady=10, sticky='ew')
        
    def create_move_relative_at_speed_button(self):
        move_rel_button = ttk.Button(self.master, text="Relative at RPM", command=self.move_relative_at_speed)
        move_rel_button.grid(row=14, column=1, padx=10, pady=10)
        
    def create_move_relative_in_time_button(self):
        move_rel_time_button = ttk.Button(self.master, text="Relative Pos in Time", command=self.move_relative_in_time)
        move_rel_time_button.grid(row=14, column=6, padx=10, pady=10)      
       

        # Add buttons to set and get PID gains
        get_button = ttk.Button(self.master, text="Get PID Gains", command=self.get_pid_gains)
        get_button.grid(row=3, column=3, columnspan=2, padx=10, pady=10, sticky='ew')

        set_button = ttk.Button(self.master, text="Set PID Gains", command=self.set_pid_gains)
        set_button.grid(row=5, column=3, columnspan=2, padx=10, pady=10, sticky='ew')

       
    def create_flash_pid_button(self):
        flash_button = ttk.Button(self.master, text="FLASH PID Gains", command=self.write_pid_gains)
        flash_button.grid(row=16, column=3, columnspan=2, pady=10, sticky='ew')
        
    def create_reload_defaults_button(self):
        style = ttk.Style()
        style.configure("Red.TButton", foreground="red")
        
        reload_button = ttk.Button(self.master, text="Reload Factory Defaults", 
                                   command=self.reload_factory_defaults, 
                                   style="Red.TButton")
        reload_button.grid(row=17, column=0, columnspan=2, padx=10, pady=10)
               
            
    def create_exit_button(self):
        style = ttk.Style()
        style.configure("Red.TButton", foreground="red")
        exit_button = ttk.Button(self.master, text="Exit", command=self.exit_application, style="Red.TButton")
        exit_button.grid(row=17, column=7, padx=20, pady=10, sticky='w')


    # Function to get PID gains via I2C
    def get_pid_gains(self):
        try:
            kp_data = self.motor_controller.miranda_read(self.motor_controller.dev_addr, 0x0D, 2)
            ki_data = self.motor_controller.miranda_read(self.motor_controller.dev_addr, 0x0F, 2)
            kd_data = self.motor_controller.miranda_read(self.motor_controller.dev_addr, 0x11, 2)
            kc_data = self.motor_controller.miranda_read(self.motor_controller.dev_addr, 0x47, 2)
        
            # Ensure data is received correctly
            if kp_data is not None and ki_data is not None and kd_data is not None and kc_data is not None:
            
                kp = (kp_data[0] << 8) | kp_data[1]
                ki = (ki_data[0] << 8) | ki_data[1]
                kd = (kd_data[0] << 8) | kd_data[1]
                kc = (kc_data[0] << 8) | kc_data[1]
        
                # Convert to float based on 0.005 resolution
                kp, ki, kd, kc = [x * 0.005 for x in (kp, ki, kd, kc)]

                # Update entry widgets
                for entry, value in zip([self.kp_entry, self.ki_entry, self.kd_entry, self.kc_entry], [kp, ki, kd, kc]):
                    entry.delete(0, tk.END)
                    entry.insert(0, "{:.4f}".format(value))

        except Exception as e:
            print(f"Error in getting gains: {e}")

    # Function to set PID gains via I2C
    def set_pid_gains(self):
        try:
            kp = float(kp_entry.get())
            ki = float(ki_entry.get())
            kd = float(kd_entry.get())
            kc = float(kc_entry.get())
        
            # Convert to fixed-point
            gains = [int(gain / 0.005) for gain in (kp, ki, kd, kc)]

            # Generate MSB and LSB pairs for each gain
            msb_lsb_pairs = [((gain >> 8) & 0xFF, gain & 0xFF) for gain in gains]

       
        except ValueError:
            print("Please enter valid numbers for gains")
        except Exception as e:
            print(f"Error in setting gains: {e}")
     
    #Dynamic Trajectory Mode allows the motor to accept new move commands while in the middle of a current move        
    def get_dtm_status(self):
        try:
            dtm_status = self.motor_controller.miranda_read(self.motor_controller.dev_addr, 0x84, 1)
            if dtm_status:
                # Assuming dtm_status[0] contains the DTM status byte
                dtm_state = "Yes" if dtm_status[0] == 0x01 else "No"
                self.dtm_output.configure(state="normal")
                self.dtm_output.delete(0, 'end')
                self.dtm_output.insert(0, dtm_state)
                self.dtm_output.configure(state="readonly")
            else:
                messagebox.showerror("Error", "Failed to read DTM status")
        except Exception as e:
            messagebox.showerror("Error", f"Error reading DTM status: {e}")
            
    def toggle_dtm(self):
        try:
            # Get the current DTM status
            current_status = self.motor_controller.miranda_read(self.motor_controller.dev_addr, 0x84, 1)
            if current_status is None:
                messagebox.showerror("Error", "Failed to read current DTM status")
                return

            # Determine the new DTM status to set
            new_status = 0x00 if current_status[0] == 0x01 else 0x01

            # Send command to set new DTM status
            self.motor_controller.miranda_write(self.motor_controller.dev_addr, 0x83, [new_status])

            # Update the DTM status display
            self.get_dtm_status()
        except Exception as e:
            messagebox.showerror("Error", f"Error toggling DTM: {e}")

   

    # Function to stop the motor
    def stop_motor(self):
        self.motor_running
        if self.motor_running:
            self.motor_running = False
            self.text_var.set("Motor stopped")
            self.motor_status_label.config(foreground='red')  # Set text color to red
            tx_data = [0x00, 0x00]  # 0 RPM
            self.motor_controller.miranda_write(self.motor_controller.dev_addr, 0x07, tx_data)

    # Function to start the motor
    def start_motor(self):
        self.motor_running

        rpm = self.rpm_entry.get().strip()

        if not rpm:
            while rpm is None:
                rpm = simpledialog.askfloat("Input", "Enter RPM")   

                if not rpm:  
                    messagebox.showerror("Error", "Invalid Input")
                    return  

                self.rpm_entry.delete(0, 'end')
                self.rpm_entry.insert(0, str(rpm))

        if rpm:  
            rpm = float(rpm)
        else:
            messagebox.showerror("Error", "Invalid RPM") 
            return

        msb, lsb = self.rpm_to_counts(rpm)

        tx_data = [msb, lsb]

        self.motor_running = True
        self.text_var.set("Motor running")
        self.motor_status_label.config(foreground='green')  # Set text color to green       
        self.motor_controller.miranda_write(self.motor_controller.dev_addr, 0x07, tx_data)

    
    def rpm_to_counts(self, rpm):
        deg_per_sec = rpm * 6  # Convert RPM to deg/sec

        # Calculate motor counts based on the conversion factor
        motor_counts = int(deg_per_sec * self.conversion_factor)

        # Handle 16-bit signed integer conversion for negative values
        if motor_counts < 0:
            motor_counts = (1 << 16) + motor_counts  #  chatgpt thinking
            #  motor_counts = 0xFFFF + motor_counts + 1  # original

        # Calculate MSB and LSB
        msb = (motor_counts >> 8) & 0xFF
        lsb = motor_counts & 0xFF

        return msb, lsb
    
    def get_acceleration(self):
        accel_data = self.motor_controller.miranda_read(self.motor_controller.dev_addr, 0x0B, 2)
        if accel_data is not None:
            acceleration = (accel_data[0] << 8) | accel_data[1]
            return acceleration
        else:
            return None  # or some default value or error handling
    
    # Function to get current encoder position
    def get_encoder_pos(self):
        pos_data = self.motor_controller.miranda_read(self.motor_controller.dev_addr, 0x1E, 2)
        if pos_data is not None:
            return (pos_data[0] << 8) | pos_data[1]
        else:
            return None

    # Function to convert encoder value to angle
    def encoder_to_angle(self, encoder_value):
        counts_per_degree = 182  # Number of motor counts per degree
        angle = encoder_value / counts_per_degree  # Convert counts to degrees
        formatted_angle = f"{angle:.2f}"  # Format to 3 decimal places
        return formatted_angle

    # Get current encoder position
    def update_encoder_display(self):
        
        curr_pos = self.get_encoder_pos()
        
        # Check if curr_pos is None
        if curr_pos is None:
            # Schedule to run again after 50ms, hoping for valid data next time
            self.master.after(50, self.update_encoder_display)
            return

        # Check if changed more than 2 counts 
        if self.prev_encoder_pos is None or abs(curr_pos - self.prev_encoder_pos) > 2:
            angle = self.encoder_to_angle(curr_pos)  # Convert encoder position to angle
            # Update entry widget
            self.encoder_entry.delete(0, 'end') 
            self.encoder_entry.insert(0, str(curr_pos))
            
            # Update angle entry widget
            self.angle_entry.delete(0, 'end')
            self.angle_entry.insert(0, str(angle))

            # Update previous value
            self.prev_encoder_pos = curr_pos
    
        # Schedule to run again after 100ms
        self.master.after(50, self.update_encoder_display)

    def set_acceleration(self):
        try:
            new_accel = int(self.accel_setting_entry.get())
            if 1 <= new_accel <= 256:
                # Convert to MSB and LSB
                msb = (new_accel >> 8) & 0xFF
                lsb = new_accel & 0xFF
                self.motor_controller.miranda_write(self.motor_controller.dev_addr, 0x08, [msb, lsb])

                # Update the acceleration display
                self.accel_entry.delete(0, 'end')
                self.accel_entry.insert(0, str(new_accel))
            else:
                print("Acceleration value out of range")
        except ValueError:
            print("Invalid input for acceleration")
            
            
    # Encoder Position 1 (home position) of 65536
    def home_motor(self):
        msb = 0x00
        lsb = 0x01

        # Send command to go to absolute location 0
        self.motor_controller.miranda_write(self.motor_controller.dev_addr, 0x05, [msb, lsb])
    

    # Function to Write PID gains to FLASH
    def write_pid_gains(self):
        self.motor_controller.miranda_write(self.motor_controller.dev_addr, 0x23, [0, 0])
        # Wait for 2000ms (2 seconds) to allow the motor controller to reset
        time.sleep(2)
        
        # Show a message box indicating successful operation
        messagebox.showinfo("Success", "PID gains successfully flashed to the motor controller.")

        
     # Reloads Factory PID, endstop, presence and sleep mode settings then resets the motor   
    def reload_factory_defaults(self):
        answer = messagebox.askyesno("Reload Factory Defaults", 
                                 "Are you sure you want to reload factory defaults? This will reset the motor.")
        if answer:
            # Send command to reload factory defaults
            self.motor_controller.miranda_write(self.motor_controller.dev_addr, 0x24, [])

            # Wait 2 seconds to ensure motor has reset before issuing new commands
            self.master.after(2000, self.post_factory_reset)
            
    def move_abs_at_rpm(self):
        # Validate and get angle
        angle_input = self.manual_angle_entry.get().strip()
        if not angle_input:
            messagebox.showerror("Error", "Invalid Angle")
            return
            self.manual_angle_entry.delete(0, 'end')
            self.manual_angle_entry.insert(0, str(angle_input))
    
        # Validate and get RPM
        rpm_input = self.rpm_entry.get().strip()
        if not rpm_input:
            messagebox.showerror("Error", "Invalid RPM")
            return
            self.rpm_entry.delete(0, 'end')
            self.rpm_entry.insert(0, str(rpm_input))

        try:
            angle = float(angle_input)
            rpm = float(rpm_input)
        except ValueError:
            messagebox.showerror("Error", "Invalid numerical input")
            return

        # Convert angle to motor counts and RPM to velocity
        motor_counts = int(angle * 182)  # 1 degree = 182 motor counts
        velocity = int(rpm * 182 / 0.022)  # Conversion as per your specification
        
        # Convert RPM to degrees per second (dps)
        degrees_per_sec = rpm * 360 / 60  # Convert RPM to degrees per second
        
        #Convert degrees per second to counts per second
        counts_per_sec = int(degrees_per_sec / 0.022)

        # Ensure velocity is within valid range
        counts_per_sec = max(min(counts_per_sec, 0x5FFF), -0x5FFF)

        # Prepare command data
        move_msb = (motor_counts >> 8) & 0xFF
        move_lsb = motor_counts & 0xFF
        vel_msb = (counts_per_sec >> 8) & 0xFF
        vel_lsb = counts_per_sec & 0xFF

        # Send command to go to absolute position at speed
        self.motor_controller.miranda_write(self.motor_controller.dev_addr, 0x42, [move_msb, move_lsb, vel_msb, vel_lsb])

    def move_abs_in_set_time(self):
        # Validate and get angle
        angle_input = self.manual_angle_entry.get().strip()
        if not angle_input:
            messagebox.showerror("Error", "Invalid Angle")
            return

        # Validate and get time
        time_input = self.time_entry.get().strip()
        if not time_input:
            messagebox.showerror("Error", "Invalid Time")
            return

        try:
            angle = float(angle_input)
            time_in_seconds = int(time_input)
        except ValueError:
            messagebox.showerror("Error", "Invalid numerical input")
            return

        # Convert angle to motor counts
        motor_counts = int(angle * 182)  # 1 degree = 182 motor counts

        # Ensure time is within valid range
        time_in_seconds = max(min(time_in_seconds, 255), 0)

        # Prepare command data
        move_msb = (motor_counts >> 8) & 0xFF
        move_lsb = motor_counts & 0xFF
        time_byte = time_in_seconds & 0xFF  # Time in seconds

        # Send command to go to absolute position in set time
        self.motor_running = True
        self.text_var.set("Motor running")
        self.motor_status_label.config(foreground='green')  # Set text color to green
        self.motor_controller.miranda_write(self.motor_controller.dev_addr, 0x09, [move_msb, move_lsb, time_byte])
        # Schedule an update to the motor status after the expected move duration
        self.master.after(int(time_in_seconds*1000), self.update_motor_status_after_move)
        
    def move_relative_at_speed(self):
        # Validate and get angle
        angle_input = self.manual_angle_entry.get().strip()
        if not angle_input:
            messagebox.showerror("Error", "Invalid Angle")
            return

        # Validate and get RPM
        rpm_input = self.rpm_entry.get().strip()
        if not rpm_input:
            messagebox.showerror("Error", "Invalid RPM")
            return

        try:
            angle = float(angle_input)
            rpm = float(rpm_input)
        except ValueError:
            messagebox.showerror("Error", "Invalid numerical input")
            return

        # Convert angle to motor counts and RPM to velocity
        motor_counts = int(angle * 182)  # 1 degree = 182 motor counts

        # Convert RPM to degrees per second (dps)
        degrees_per_sec = rpm * 360 / 60  # Convert RPM to degrees per second
    
        # Convert degrees per second to counts per second
        counts_per_sec = int(degrees_per_sec / 0.022)

        # Ensure velocity is within valid range
        counts_per_sec = max(min(counts_per_sec, 0x5FFF), -0x5FFF)

        # Prepare command data
        move_msb = (motor_counts >> 8) & 0xFF
        move_lsb = motor_counts & 0xFF
        vel_msb = (counts_per_sec >> 8) & 0xFF
        vel_lsb = counts_per_sec & 0xFF

        # Send command to go to relative position at speed
        self.motor_controller.miranda_write(self.motor_controller.dev_addr, 0x41, [move_msb, move_lsb, vel_msb, vel_lsb])
        
    def move_relative_in_time(self):
        # Validate and get angle
        angle_input = self.manual_angle_entry.get().strip()
        if not angle_input:
            messagebox.showerror("Error", "Invalid Angle")
            return

        # Validate and get time
        time_input = self.time_entry.get().strip()
        if not time_input:
            messagebox.showerror("Error", "Invalid Time")
            return

        try:
            angle = float(angle_input)
            time_in_seconds = float(time_input)
        except ValueError:
            messagebox.showerror("Error", "Invalid numerical input")
            return

        # Convert angle to motor counts
        motor_counts = int(angle * 182)  # 1 degree = 182 motor counts

        # Convert time to milliseconds (hundredths of a second)
        time_in_ms = int(time_in_seconds * 100)

        # Ensure time is within valid range
        time_in_ms = max(min(time_in_ms, 0xFFFF), 0)

        # Determine direction based on angle sign
        direction = 0x00 if angle >= 0 else 0x01

        # Prepare command data
        move_msb = (abs(motor_counts) >> 8) & 0xFF
        move_lsb = abs(motor_counts) & 0xFF
        time_msb = (time_in_ms >> 8) & 0xFF
        time_lsb = time_in_ms & 0xFF
        
        
        # Send command for relative position in time
        self.motor_running = True
        self.text_var.set("Motor running")
        self.motor_status_label.config(foreground='green')  # Set text color to green
        self.motor_controller.miranda_write(self.motor_controller.dev_addr, 0x5F, [move_msb, move_lsb, time_msb, time_lsb, direction])
        # Schedule an update to the motor status after the expected move duration
        self.master.after(int(time_in_seconds*1000), self.update_motor_status_after_move)
        
    def update_motor_status_after_move(self):
        if self.motor_running:
            self.motor_running = False
            self.text_var.set("Motor stopped")
            self.motor_status_label.config(foreground='red')  # Set text color to red
    
            
    def post_factory_reset(self):
        # Update the encoder display
        self.update_encoder_display()

        # Update the acceleration display
        current_acceleration = self.get_acceleration()
        if current_acceleration is not None:
            self.accel_entry.delete(0, 'end')
            self.accel_entry.insert(0, str(current_acceleration))

        # Reload PID values
        self.get_pid_gains()

        # Update the motor status label to indicate that a factory reset was performed
        self.text_var.set("Factory Reset Performed")

        # a delay or scheduling a revert to the default status message after some time
        self.master.after(5000, lambda: self.text_var.set("Motor stopped"))

    
    # Stop the motor if it's running and Exits the App   
    def exit_application(self):
        answer = messagebox.askyesno("Exit", "Are you sure you want to Exit? Did you FLASH the controller?")
        if answer:
            if self.motor_running:
                self.stop_motor()  # Make sure this method safely stops the motor

            try:
                self.master.destroy()  # Close the GUI
            except:
                pass  # Ignore if the window is already closed 
                

if __name__ == "__main__":
    root = tk.Tk()
    motor_controller = MotorController()  # Assuming MotorController class is defined as earlier
    tuner = MotorTuner(root, motor_controller)
    root.protocol("WM_DELETE_WINDOW", tuner.exit_application)  # Handle window close button
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        # Handle the Ctrl+C keyboard interrupt
        tuner.exit_application()

