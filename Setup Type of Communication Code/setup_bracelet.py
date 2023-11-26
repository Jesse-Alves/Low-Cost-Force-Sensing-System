#     /////////////////////////////////////////////////////////////////
#    //        BRACELET TO MEASURE THE UPPER-ARM CONTRACTION        //
#   //           Author: Jesse de Oliveira Santana Alves           //
#  //                 Email: jessalves2@gmail.com                 //
# /////////////////////////////////////////////////////////////////


# =======================================================
# ====================>  Libraries  <====================
# =======================================================
import tkinter
import tkinter.messagebox
import customtkinter
from tkinter import font
import serial
import serial.tools.list_ports
import time

# Setup initial parameters
customtkinter.set_appearance_mode("System")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"


class Setup(customtkinter.CTk):
    def __init__(self):
        super().__init__()

        # =======================================================
        # =================>  Initial Values  <==================
        # =======================================================
        self.type_protocol = "not configured"
        self.ssid = "none"
        self.password = "none"

        # Font
        self.my_font = customtkinter.CTkFont(family="Arial", size=25)

        # configure window
        self.title("Setup the Bracelet Communication Protocol")
        self.geometry(f"{900}x{580}")

        # configure grid layout (4x4)
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure((2, 3), weight=0)
        self.grid_rowconfigure((0, 1, 2), weight=1)

        # =======================================================
        # ====================>  Side Bar  <=====================
        # =======================================================

        # create sidebar frame with widgets
        self.sidebar_frame = customtkinter.CTkFrame(self, width=140, corner_radius=0)
        self.sidebar_frame.grid(row=0, column=0, rowspan=4, sticky="nsew")
        self.sidebar_frame.grid_rowconfigure(4, weight=1)
        
        self.logo_label = customtkinter.CTkLabel(self.sidebar_frame, text="Setup Bracelet", font=customtkinter.CTkFont(size=20, weight="bold"))
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))
                
        self.appearance_mode_label = customtkinter.CTkLabel(self.sidebar_frame, text="Appearance Mode:", anchor="w")
        self.appearance_mode_label.grid(row=5, column=0, padx=20, pady=(10, 0))
        self.appearance_mode_optionemenu = customtkinter.CTkOptionMenu(self.sidebar_frame, values=["Light", "Dark", "System"],
                                                                       command=self.change_appearance_mode_event)
        self.appearance_mode_optionemenu.grid(row=6, column=0, padx=20, pady=(10, 10))
        self.scaling_label = customtkinter.CTkLabel(self.sidebar_frame, text="UI Scaling:", anchor="w")
        self.scaling_label.grid(row=7, column=0, padx=20, pady=(10, 0))
        self.scaling_optionemenu = customtkinter.CTkOptionMenu(self.sidebar_frame, values=["80%", "90%", "100%", "110%", "120%"],
                                                               command=self.change_scaling_event)
        self.scaling_optionemenu.grid(row=8, column=0, padx=20, pady=(10, 20))

        self.label_uconnect = customtkinter.CTkLabel(self.sidebar_frame, text=" ", font=customtkinter.CTkFont(size=12, weight="bold"))
        self.label_uconnect.grid(row=9, column=0, padx=20, pady=(20, 10))  

        # =======================================================
        # ==================>  First Frame  <====================
        # =======================================================       
        self.label1_frame = customtkinter.CTkFrame(self, fg_color="transparent")
        self.label1_frame.grid(row=0, column=1, padx=(0, 0), pady=(0, 0), sticky="nsew")
        self.label1_frame.grid_columnconfigure(0, weight=2)
        self.label1_frame.grid_rowconfigure(4, weight=2)
        self.label1 = customtkinter.CTkLabel(self.label1_frame, text=" ", font=customtkinter.CTkFont(size=25, weight="bold"))
        self.label1.grid(row=4, column=0, padx=20, pady=(20, 10))  

        # =======================================================
        # ==================>  Second Frame  <===================
        # =======================================================          
        self.frame2 = customtkinter.CTkFrame(self, fg_color="transparent")
        self.frame2.grid(row=1, column=1, padx=(0, 0), pady=(0, 0), sticky="nsew")
        self.frame2.grid_columnconfigure(0, weight=2)
        self.frame2.grid_rowconfigure(4, weight=2)


        # =======================================================
        # ==================>  Main Logic  <=====================
        # =======================================================

        # 1 - Check if the USB Microcontroller was connected
        microcontroller_port = self.get_microcontroller_port()

        if (microcontroller_port == None):
            # =======> Window 1 - Waiting the Micro Connection    
            self.label1.configure(text="Please, plug the microcontroller \n into the computer's USB port!", 
                                  font=customtkinter.CTkFont(size=25, weight="bold"))
            self.label1.grid(row=4, column=0, padx=20, pady=(20, 10))  

            # Create a Loading Bar
            self.progressbar_1 = customtkinter.CTkProgressBar(self.frame2)
            self.progressbar_1.grid(row=1, column=0, padx=(20, 10), pady=(10, 10), sticky="ew")
            self.progressbar_1.configure(mode="indeterminnate")
            self.progressbar_1.start()

            # Create Timer to check the micro connection          
            time = 500
            self.timer = self.after(time, self.check_micro_connection)
        else:
            # When the microcontroller is connected
            self.set_window2()
            
        # =======================================================
        # ===============>  Set Default Values  <================
        # =======================================================
        self.appearance_mode_optionemenu.set("Dark")
        self.scaling_optionemenu.set("120%")
        self.change_scaling_event("120%")        
    # =======================================================
    # ====================>  Functions  <====================
    # =======================================================
    def get_microcontroller_port(self):
        ports = serial.tools.list_ports.comports()
        microcontroller_port = None
        
        for port in ports:
            if "USB" in port.description:  # Modify this condition to match your microcontroller's description
                microcontroller_port = port.device
                break
        
        return microcontroller_port

    def check_micro_connection(self):
        # Set again the timer - It is necessary do it to keep the callback checking
        self.timer = self.after(500, self.check_micro_connection)

        # 1 - Check if the USB Microcontroller was connected
        microcontroller_port = self.get_microcontroller_port()
        #print("checking")

        if (microcontroller_port == None):
            pass
        else:
            # Stop the Callback function
            self.after_cancel(self.timer)

            # Remove the loading bar    
            self.progressbar_1.destroy()
            
            # Skip to Window 2
            self.set_window2()
            
    def set_window2(self):
        # Label to indicate that the Microcontroller is connected
        microcontroller_port = self.get_microcontroller_port()
        self.label_uconnect.configure(text= f"Microcontroller Connected in \n {microcontroller_port} port!")

        # =======================================================
        # ====================>  Label 1  <======================
        # =======================================================  
        self.label1_frame.grid_rowconfigure(1, weight=2)        
        self.label1.configure(text="Configure the type of communication")
        self.label1.grid(row=1, column=0, padx=20, pady=(20, 10))

        # =======================================================
        # ================>  Radio Buttons 1  <==================
        # =======================================================  
        
        # Title
        self.label_radio_group = customtkinter.CTkLabel(master=self.frame2, text="Select the type:", 
                                                        font=customtkinter.CTkFont(size=18, weight="bold"))
        self.label_radio_group.grid(row=0, column=0, columnspan=1, padx=30, pady=10, sticky="nw")

        # Radio button to selec the protocol communication
        #self.radio_var = tkinter.IntVar(value=0)
        self.type_protocolGUI = tkinter.StringVar()

        self.radio_button_1 = customtkinter.CTkRadioButton(master=self.frame2, text= "USB Connection", 
                                                           variable=self.type_protocolGUI, value="serial", 
                                                           command=self.hide_frame3)        
        self.radio_button_1.grid(row=1, column=0, pady=10, padx=40, sticky="nw")

        self.radio_button_2 = customtkinter.CTkRadioButton(master=self.frame2, text= "Wifi and USB Connection", 
                                                           variable=self.type_protocolGUI, 
                                                           value="wifi", command=self.show_frame3)
        self.radio_button_2.grid(row=2, column=0, pady=10, padx=40, sticky="nw")


        # =======================================================
        # ====================> Third Frame  <===================   // Wifi Info Frame
        # =======================================================  
        self.frame3 = customtkinter.CTkFrame(self, fg_color="transparent")
        self.frame3.grid(row=2, column=1, padx=(0, 0), pady=(0, 0), sticky="nsew")
        self.frame3.grid_columnconfigure(0, weight=2)
        self.frame3.grid_rowconfigure(4, weight=2)

        # =======================================================
        # =================>  INSTALL BUTTON  <==================
        # =======================================================        
        self.main_button_1 = customtkinter.CTkButton(master=self, fg_color="transparent", 
                                                     border_width=3, text_color=("gray10", "#DCE4EE"), 
                                                     text="Install", font=self.my_font,
                                                     command=self.Install_button)
        
        self.main_button_1.grid(row=3, column=1, padx=(10, 10), pady=(10, 10), sticky="e")

    def set_window3(self):
        self.frame2.destroy()
        self.frame3.destroy()
        self.label1.configure(text="Setup completed successfully!", 
                                font=customtkinter.CTkFont(size=30, weight="bold"))

        # Close the Window Automatically
        self.main_button_1.configure(text="Finish", command=self.close_setup)

    def send_data_to_microcontroller(self):

        # Check if the microcontroller still connected
        microcontroller_port = self.get_microcontroller_port()
        if (microcontroller_port == None):
            self.label1.configure(text="Please, connect the microcontroller \ninto the computer's USB port again \n before to continue!")
        else:                        
            try:    
                # Connect to a USB port to exchange data
                ser = serial.Serial(microcontroller_port, 115200)
                
                # =======================================> Send protocol type
                ser.write(self.type_protocol.encode())
                time.sleep(0.2)
                
                # Confirmation
                self.confirmation_txt = ser.readline()
                self.confirmation_txt = self.confirmation_txt.decode('utf-8')
                print(f"The reponse was: {self.confirmation_txt}")
                
                # =======================================> Send ssid wifi
                ser.write(self.ssid.encode())
                time.sleep(0.2)
                
                # Confirmation
                self.confirmation_txt = ser.readline()
                self.confirmation_txt = self.confirmation_txt.decode('utf-8')
                print(f"The reponse was: {self.confirmation_txt}")
                
                # =======================================> Send password wifi
                ser.write(self.password.encode())
                time.sleep(0.2)
                
                # Confirmation
                self.confirmation_txt = ser.readline()
                self.confirmation_txt = self.confirmation_txt.decode('utf-8')
                print(f"The reponse was: {self.confirmation_txt}")
                
                # =======================================> Confirm that all data was sent
                time.sleep(10)
                self.set_window3()

            except:
                self.label1.configure(text="ERROR: It looks like the microcontroller is connected \n to another program, perhaps the Arduino IDE. \nPlease close this program before to continue!", 
                                      font=customtkinter.CTkFont(size=18, weight="bold"))

    # =======================================================
    # ==================>  Event Functions  <================
    # =======================================================
    def change_appearance_mode_event(self, new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)
    def change_scaling_event(self, new_scaling: str):
        new_scaling_float = int(new_scaling.replace("%", "")) / 100
        customtkinter.set_widget_scaling(new_scaling_float)
    def show_frame3(self):
        self.label2 = customtkinter.CTkLabel(self.frame3, text="Select a Network:", 
                                            font=customtkinter.CTkFont(size=18, weight="bold"))
        self.label2.grid(row=0, column=0,  padx=30, pady=10, sticky="w")  

        self.ssidGUI = customtkinter.CTkEntry(self.frame3, placeholder_text="Network name (ssid)")
        self.ssidGUI.grid(row=1, column=0, padx=40, pady=10,  sticky="nsew")

        self.passwordGUI = customtkinter.CTkEntry(self.frame3, placeholder_text="Password")
        self.passwordGUI.grid(row=2, column=0, padx=40, pady=10,  sticky="nsew")
    def hide_frame3(self):
        try:
            self.label2.destroy()
            self.ssidGUI.destroy()
            self.passwordGUI.destroy()
        except:
            pass
    def Install_button(self):      
        # ===========> Check if all the fileds were filled
        self.type_protocol = self.type_protocolGUI.get()

        if (self.type_protocol == "serial"): 
            self.send_data_to_microcontroller()

        elif (self.type_protocol == "wifi"):
            self.ssid = self.ssidGUI.get()
            self.password = self.passwordGUI.get()

            if (self.ssid == "") or (self.password == ""):                
                self.label1.configure(text="Please, type the Network Information\n before to continue!")
            else:
                self.send_data_to_microcontroller()
        else:
            # Ask to select any protocol type
            self.label1.configure(text="Please, select one type of \n communication before to continue!")
    def close_setup(self):
        self.destroy()

if __name__ == "__main__":
    setup_app = Setup()
    setup_app.mainloop()