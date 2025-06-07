import os
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import math
import pandas as pd

# Model for simulation logic and experiment data
class LauncherModel:
    def __init__(self):
        self.experimental_data = self.load_experiment_data()

    # Calculate tilt angle in degrees from robot at (robot_x, robot_y) to (x, y)
    def tilt_for(self, x, y, robot_x=-0.495, robot_y=1.00):
        tilt_rad = math.atan2(y - robot_y, x - robot_x)
        tilt_deg = math.degrees(tilt_rad)
        return round(tilt_deg, 2)

    # Load experiment data: average X, Y for each angle (sheet name)
    def load_experiment_data(self):
        excel_path = 'ทดลองเปลี่ยนองศา (3_6_68).xlsx'
        all_sheets = pd.read_excel(excel_path, sheet_name=None)
        data = {}
        for sheet_name, df in all_sheets.items():
            try:
                angle = float(sheet_name)
            except ValueError:
                continue  # Skip sheets that are not angles

            # Use mean of all attempts for this angle
            landing_x = df['แกน X'].mean()
            landing_y = df['แกน Y'].mean()
            # If Pressure column exists, use mean, else None
            pressure = df['Pressure'].mean() if 'Pressure' in df.columns else None

            data[angle] = (landing_x, landing_y, pressure)
        return data

    # Find the nearest experiment data for a given angle
    def get_nearest_experiment(self, angle):
        available_angles = list(self.experimental_data.keys())
        nearest_angle = min(available_angles, key=lambda a: abs(a - angle))
        return nearest_angle, self.experimental_data[nearest_angle]

    # Calculate pressure based on projectile motion physics
    def pressure_for(self, distance, height, angle_deg, g=9.81, k=0.05):
        theta = math.radians(angle_deg)
        if angle_deg == 90 or math.isclose(math.cos(theta), 0):
            return None  # Can't shoot vertically

        try:
            A = math.tan(theta)
            B = height - distance * A
            if B <= 0:
                return None  # Not physically possible
            v = distance / (math.cos(theta) * math.sqrt((2 * B) / g))
            if not math.isfinite(v):
                return None
        except Exception:
            return None

        pressure = k * v**2
        return round(pressure, 2)

    # Get landing position from experiment data (for marker)
    def landing_for(self, angle):
        nearest_angle, (exp_x, exp_y, _) = self.get_nearest_experiment(angle)
        return exp_x, exp_y

# Store simulation history
class HistoryLog:
    def __init__(self):
        self.entries = []

    def add_entry(self, inputs, outputs):
        self.entries.append((inputs, outputs))

    def clear(self):
        self.entries.clear()

    def get_all(self):
        return self.entries

# Main application class for the GUI
class LauncherApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Squash Launcher GUI")
        self.geometry("1366x768")
        self.resizable(False, False)

        # Load background images
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.bg_setup = ImageTk.PhotoImage(Image.open(os.path.join(base_dir, "Setup page.jpg")).resize((1366, 768)))
        self.bg_result = ImageTk.PhotoImage(Image.open(os.path.join(base_dir, "Result page.jpg")).resize((1366, 768)))
        self.bg_history = ImageTk.PhotoImage(Image.open(os.path.join(base_dir, "History page.jpg")).resize((1366, 768)))

        # Initialize model and history
        self.model = LauncherModel()
        self.history_log = HistoryLog()

        # Set up container for pages
        container = tk.Frame(self)
        container.pack(fill="both", expand=True)

        # Create and store all pages
        self.frames = {}
        for Page in (InputPage, OutputPage, HistoryPage):
            frame = Page(parent=container, controller=self)
            self.frames[Page.__name__] = frame
            frame.place(x=0, y=0, relwidth=1, relheight=1)

        self.show_frame("InputPage")

    # Show the requested page
    def show_frame(self, page_name):
        self.frames[page_name].tkraise()

    # Run the simulation and update output/history
    def run_simulation(self, inputs):
        x, y, height, angle = inputs['x'], inputs['y'], inputs['height'], inputs['angle']
        # Calculate distance from robot's X to target X
        robot_x = -0.495
        distance = x - robot_x
        pressure = self.model.pressure_for(distance, height, angle)
        tilt = self.model.tilt_for(x, y, robot_x=robot_x, robot_y=1.00)

        # Use user input for marker position
        outputs = {'pressure': pressure, 'tilt': tilt, 'landing': (x, y)}
        self.history_log.add_entry(inputs, outputs)
        self.frames["OutputPage"].display_results(pressure, tilt, (x, y))
        self.show_frame("OutputPage")

    def save_history(self):
        pass

    def get_history(self):
        return self.history_log.get_all()

    def clear_history(self):
        self.history_log.clear()
        self.frames["HistoryPage"].update_table(self.get_history())

# Input page for user parameters
class InputPage(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller
        self.canvas = tk.Canvas(self, width=1366, height=768)
        self.canvas.pack()
        self.canvas.create_image(0, 0, image=controller.bg_setup, anchor="nw")

        # Input fields for X, Y, Height, Angle
        self.x_entry = tk.Entry(self, font=("Arial", 14))
        self.y_entry = tk.Entry(self, font=("Arial", 14))
        self.height_entry = tk.Entry(self, font=("Arial", 14))
        self.angle_entry = tk.Entry(self, font=("Arial", 14))

        # Place input fields on the canvas
        self.canvas.create_window(573, 105, window=self.x_entry, width=100, height=25)
        self.canvas.create_window(573, 155, window=self.y_entry, width=100, height=25)
        self.canvas.create_window(940, 155, window=self.height_entry, width=60, height=25)
        self.canvas.create_window(1180, 155, window=self.angle_entry, width=60, height=25)

        # Confirm and History buttons
        self.sim_btn = tk.Button(self, text="Confirm", font=("Arial", 14), bg="gray", command=self.submit)
        self.canvas.create_window(769, 705, window=self.sim_btn, width=270, height=100)

        self.hist_btn = tk.Button(self, text="History", font=("Arial", 12), command=lambda: controller.show_frame("HistoryPage"))
        self.canvas.create_window(98, 733, window=self.hist_btn, width=70, height=55)

    # Validate user input and show warnings if out of range
    def validate_inputs(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            height = float(self.height_entry.get())
            angle = float(self.angle_entry.get())

            # Range checks for all parameters
            if not (0 <= x <= 4.45):
                messagebox.showerror("Input Error", "X must be between 0 and 4.45 meters.")
                return None
            if not (0 <= y <= 2.00):
                messagebox.showerror("Input Error", "Y must be between 0 and 2.00 meters.")
                return None
            if not (0 < height <= 2.00):
                messagebox.showerror("Input Error", "Height must be greater than 0 and less than or equal to 2.00 meters.")
                return None
            if not (0 <= angle <= 90):
                messagebox.showerror("Input Error", "Angle must be between 0 and 90 degrees.")
                return None

            return {'x': x, 'y': y, 'height': height, 'angle': angle}
        except ValueError:
            messagebox.showerror("Input Error", "Please enter valid numbers.")
            return None

    # Submit input and run simulation
    def submit(self):
        inputs = self.validate_inputs()
        if inputs:
            self.controller.run_simulation(inputs)

# Output page to display results and landing marker
class OutputPage(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller
        self.canvas = tk.Canvas(self, width=1366, height=768)
        self.canvas.pack()
        self.canvas.create_image(0, 0, image=controller.bg_result, anchor="nw")

        self.pressure_var = tk.StringVar()
        self.tilt_var = tk.StringVar()

        # Read-only entries for pressure and tilt
        self.pressure_display = tk.Entry(self, textvariable=self.pressure_var, font=("Arial", 14), state="readonly")
        self.tilt_display = tk.Entry(self, textvariable=self.tilt_var, font=("Arial", 14), state="readonly")
        self.canvas.create_window(508, 135, window=self.pressure_display, width=100, height=25)
        self.canvas.create_window(1005, 137, window=self.tilt_display, width=100, height=25)

        # Navigation buttons
        self.back_btn = tk.Button(self, text="Reset", font=("Arial", 14), bg="orange", command=lambda: controller.show_frame("InputPage"))
        self.canvas.create_window(780, 703.6, window=self.back_btn, width=76, height=76)

        self.hist_btn = tk.Button(self, text="History", font=("Arial", 12), command=lambda: controller.show_frame("HistoryPage"))
        self.canvas.create_window(98, 733, window=self.hist_btn, width=70, height=55)

        self.landing_marker = None  # Track the marker

    # Display results and draw landing marker
    def display_results(self, pressure, tilt, landing_coord):
        self.pressure_display.config(state="normal")
        self.tilt_display.config(state="normal")
        self.pressure_var.set(pressure)
        self.tilt_var.set(tilt)
        self.pressure_display.config(state="readonly")
        self.tilt_display.config(state="readonly")
        x, y = landing_coord
        self.draw_landing_marker(x, y)

    # Draw the landing marker on the field
    def draw_landing_marker(self, x, y):
        if hasattr(self, 'landing_marker') and self.landing_marker:
            self.canvas.delete(self.landing_marker)

        # Field dimensions and scaling
        field_start_x = 290
        field_start_y = 220
        field_width_px = 815
        field_height_px = 420
        field_width_m = 4.45
        field_height_m = 2.00

        scale_x = field_width_px / field_width_m
        scale_y = field_height_px / field_height_m

        # Flip X so 0 is right and 4.45 is left
        pixel_x = field_start_x + field_width_px - (x * scale_x)
        # Flip Y so 0 is bottom and 2.00 is top
        pixel_y = field_start_y + field_height_px - (y * scale_y)

        r = 8
        self.landing_marker = self.canvas.create_oval(
            pixel_x - r, pixel_y - r, pixel_x + r, pixel_y + r,
            fill="red", outline="black", width=2
        )

# History page to show all simulation runs
class HistoryPage(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller
        self.canvas = tk.Canvas(self, width=1366, height=768)
        self.canvas.pack()
        self.canvas.create_image(0, 0, image=controller.bg_history, anchor="nw")

        columns = ("X", "Y", "Height", "Angle", "Pressure", "Tilt")
        self.tree = ttk.Treeview(self, columns=columns, show="headings")
        for col in columns:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=120)
        self.tree.place(x=100, y=100, width=1166, height=500)

        tk.Button(self, text="Clear", font=("Arial", 14), bg="red", command=controller.clear_history).place(x=500, y=650, width=100, height=40)
        tk.Button(self, text="Go Back", font=("Arial", 14), bg="orange", command=lambda: controller.show_frame("InputPage")).place(x=700, y=650, width=150, height=40)

        self.update_table(controller.get_history())

    # Update the history table with all entries
    def update_table(self, history_list):
        for row in self.tree.get_children():
            self.tree.delete(row)
        for entry, result in history_list:
            self.tree.insert("", "end", values=(
                entry['x'], entry['y'], entry['height'], entry['angle'],
                result['pressure'], result['tilt']
            ))

    def tkraise(self, *args, **kwargs):
        super().tkraise(*args, **kwargs)
        self.update_table(self.controller.get_history())

# Start the application
if __name__ == "__main__":
    app = LauncherApp()
    app.mainloop()