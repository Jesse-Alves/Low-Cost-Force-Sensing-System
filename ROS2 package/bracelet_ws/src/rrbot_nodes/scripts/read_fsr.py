#!/usr/bin/env python3

#     /////////////////////////////////////////////////////////////////
#    //        BRACELET TO MEASURE THE UPPER-ARM CONTRACTION        //
#   //           Author: Jesse de Oliveira Santana Alves           //
#  //                 Email: jessalves2@gmail.com                 //
# /////////////////////////////////////////////////////////////////


# =======================================================
# ====================>  Libraries  <====================
# =======================================================
#GUI
import tkinter
import tkinter.messagebox
import customtkinter
from tkinter import font
import threading

# Comunication
import serial
import serial.tools.list_ports
import time
import socket

#ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# Setup initial parameters
customtkinter.set_appearance_mode("System")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"


class Calibration(customtkinter.CTk):
    def __init__(self):
        super().__init__()

        # =======================================================
        # =================>  Initial Values  <==================
        # =======================================================
        self.type_protocol = " "
        self.gotit_message = "got_it"
        self.nothing_message = "nothing"

        # ==========================> Wifi Parameters
        #try:
        # bind all IP
        self.HOST = '0.0.0.0' 
        # Listen on Port 
        self.PORT = 1234 
        #Size of receive buffer   
        self.BUFFER_SIZE = 1024    
        # Create a TCP/IP socket
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind the socket to the host and port
        self.s.bind((self.HOST, self.PORT))
        #except:
        #    pass


        # Font
        self.my_font = customtkinter.CTkFont(family="Arial", size=25)

        # configure window
        self.title("Calibration Step for the Bracelet")
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
        
        self.logo_label = customtkinter.CTkLabel(self.sidebar_frame, text="Calibration", font=customtkinter.CTkFont(size=20, weight="bold"))
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


        # =======================================================================================
        # =================================>  Main Logic  <======================================
        # =======================================================================================


        # 1 - Check if the USB Microcontroller was connected
        self.period = 500
        self.microcontroller_port = self.get_microcontroller_port()

        if (self.microcontroller_port == None):
            # =======> Window 1 - Waiting the Micro Connection    
            self.label1.configure(text="The microcontroller is not connected \ninto the computer's USB port!", 
                                  font=customtkinter.CTkFont(size=25, weight="bold"))
            self.label1.grid(row=4, column=0, padx=20, pady=(20, 10))  

            # Deciding the type of communication
            self.textbox = customtkinter.CTkTextbox(self.frame2, width=250, height=300)
            self.textbox.grid(row=0, column=0, padx=(0, 0), pady=(10, 10), sticky="nsew")
            self.textbox.insert("0.0","=> If the connection will be via USB:\n 1 - Connect the USB cable into the computer. \n 2 - Then, click on USB option.\n\n" + "=> If the connection will be via Wifi:\n 1 - Make sure the microcontroller is turned on.\n 2 - Then, click on WIFI option.")
            self.textbox.configure(font= self.my_font)

            #Buttons
            self.main_button_1 = customtkinter.CTkButton(master=self, fg_color="transparent", 
                                                        border_width=3, text_color=("gray10", "#DCE4EE"), 
                                                        text="Wifi", font=self.my_font,
                                                        command=self.wifi_option)        
            self.main_button_1.grid(row=3, column=1, padx=(10, 10), pady=(10, 10), sticky="e")

            self.usb_button = customtkinter.CTkButton(master=self, fg_color="transparent", 
                                                        border_width=3, text_color=("gray10", "#DCE4EE"), 
                                                        text="USB", font=self.my_font,
                                                        command=self.usb_option)        
            self.usb_button.grid(row=3, column=1, padx=(10, 10), pady=(10, 10), sticky="w")
        else:
            #Check the type of connection sending via           
            self.timer_USB_info = self.after(self.period, self.read_USB_info)

        # =======================================================
        # ===============>  Set Default Values  <================
        # =======================================================
        self.appearance_mode_optionemenu.set("Dark")
        self.scaling_optionemenu.set("120%")
        self.change_scaling_event("120%")  

    # ============================================ END OF INIT =============================================

    # =======================================================
    # ====================>  Functions  <====================
    # =======================================================
    def read_USB_info(self):
        
        try:
            self.ser = serial.Serial(self.microcontroller_port, 115200)
            #self.ser.write(self.nothing_message.encode())
            #time.sleep(0.1)
            
            # Confirmation
            self.type_protocol = self.ser.readline()
            self.type_protocol = self.type_protocol.decode('utf-8')
            print(f"The reponse was: {self.type_protocol}")

            if "serial" in self.type_protocol:
                self.type_protocol = "serial"
                self.label_uconnect.configure(text= f"Microcontroller Connected via USB \n(port {self.microcontroller_port}).")

                #print(f"I understood that the tp is: {self.type_protocol}")
                # Confirm the receiving
                self.ser.write(self.gotit_message.encode())
                time.sleep(0.2)
                self.confirmation_txt = self.ser.readline()
                self.confirmation_txt = self.confirmation_txt.decode('utf-8')

                # Cancel the timer
                self.after_cancel(self.timer_USB_info)

                # When the microcontroller is connected
                time.sleep(0.5)
                self.set_window1()
            elif "wifi" in self.type_protocol:
                #self.type_protocol = "wifi"

                #print("Trying to connect via wifi.")
                for i in range(2):
                    self.wifi_data_received, self.client_address = self.s.recvfrom(self.BUFFER_SIZE)
                    if self.wifi_data_received:                         
                        self.wifi_data_received = self.wifi_data_received.decode('utf-8')
                        if "wifi" in self.wifi_data_received:
                            # Define the type of protocol
                            self.type_protocol = "wifi"                        
                            self.label_uconnect.configure(text= f"Microcontroller Connected via Wifi.")    

                            # Cancel the timer
                            self.after_cancel(self.timer_USB_info)
                            self.timer = self.after(500, self.first_wifi_confirmation)
                     
                            # Modify layout     
                            #time.sleep(0.5)                 
                            self.set_window1()


                # self.ser.write(self.gotit_message.encode())
                # time.sleep(0.2)
                # self.confirmation_txt = self.ser.readline()
                # self.confirmation_txt = self.confirmation_txt.decode('utf-8')

                # # Cancel the timer
                # self.after_cancel(self.timer_USB_info)

                # # When the microcontroller is connected
                # self.set_window1()
            else:
                #time.sleep(0.1)
                self.timer_USB_info = self.after(self.period, self.read_USB_info)

        except serial.SerialException as e:
            self.label1.configure(text="ERROR: It looks like the microcontroller connected \n to another program, perhaps the Arduino IDE. \nPlease close the another IDE \nand then, relaunch this python code!", 
                                font=customtkinter.CTkFont(size=18, weight="bold"))
      
    # def check_type_of_communication(self):        

    #     #try:
    #     #Connection via USB
    #     self.microcontroller_port = self.get_microcontroller_port()

    #     # Try to get data via wifi.
    #     self.wifi_data_received, self.client_address = self.s.recvfrom(self.BUFFER_SIZE)


    #     if self.wifi_data_received: #-------------------------------------------------------

    #         print("I receive something!")
    #         self.wifi_data_received = self.wifi_data_received.decode('utf-8')
    #         if "wifi" in self.wifi_data_received:
    #             # Define the type of protocol
    #             self.type_protocol = "wifi"
                

    #             # Answer the Micro, the data was received
    #             time.sleep(0.3)
    #             self.timer = self.after(500, self.first_wifi_confirmation)

    #             #self.send_via_wifi
    #             self.after_cancel(self.timer_check)

    #             # Modify layout
    #             self.progressbar_1.destroy()                        
    #             self.set_window1() 

    #     elif not (self.microcontroller_port == None): #---------------------------------------
            
    #         print("I identify the connection")
    #         # Stop verification timer
    #         self.after_cancel(self.timer_check)

    #         # Modify layout
    #         self.progressbar_1.destroy()                        
    #         self.set_window1()            

    #         #Check the type of connection sending via           
    #         self.timer_USB_info = self.after(self.period, self.read_USB_info)


    #     else: #-------------------------------------------------------------------------------
    #         #time.sleep(0.1)
    #         print("I will execute again!")
    #         self.timer_check = self.after(self.period, self.check_type_of_communication)

    #     #except:
    #         #pass

    # def check_type_of_communication(self):

    #     # # Wait the GUI load        
    #     # if (time.time() - self.start_time) < 2: 
    #     #     #print("I am not executing...")
    #     #     return

    #     #print("called the self.check_type_of_communication function")

    #     # Check if the microcontroller is connected
    #     self.microcontroller_port = self.get_microcontroller_port()
    #     print(self.microcontroller_port)
    #     if (self.microcontroller_port == None):
    #         pass            
    #     else:          
    #         try:          
    #             # Connect to a USB port to exchange data
    #             self.ser = serial.Serial(self.microcontroller_port, 115200)
                
    #             # =======================================> Send Got it
    #             #open_communication = "nothing"
    #             #self.ser.write(open_communication.encode())
    #             #time.sleep(0.1)
    #             #print("I passed")

    #             # Confirmation
    #             for i in range(2):
    #                 self.confirmation_txt = self.ser.readline()
    #                 self.confirmation_txt = self.confirmation_txt.decode('utf-8')

    #                 print(f"The confirmation message: {self.confirmation_txt}")
    #                 #print(type(self.confirmation_txt))

    #                 if "serial" in self.confirmation_txt:                  
    #                     self.type_protocol = "serial"                                            
                        
    #                     for i in range(2):
    #                         self.send_via_serial()

    #                     while(True):
    #                         self.confirmation_txt = self.ser.readline()
    #                         self.confirmation_txt = self.confirmation_txt.decode('utf-8')
    #                         print(f"The confirmation: {self.confirmation_txt}")

    #                     #time.sleep(0.3)
    #                     #print(f"type of protocol: {self.type_protocol}")

    #                     print("I will cancel the timer...")
    #                     self.after_cancel(self.timer_check_connection)
    #                     print("I canceled the timer") 

    #                     #Set the Window 1 to start calibration
    #                     self.set_window1()

    #                     # return the function
    #                     print("I will return")
    #                     return

    #                 time.sleep(0.2)
    #         except:
    #             pass

    #     #print("sair")
    #     #print(self.type_protocol)
    #     if self.type_protocol == "serial":
    #         return
    #     else:
    #         # 1 - Check 3 times if the communication is via Wifi
    #         for i in range(2):
    #             try:
    #                 # Try to get data via wifi.
    #                 self.wifi_data_received, self.client_address = self.s.recvfrom(self.BUFFER_SIZE)
    #                 if self.wifi_data_received:
    #                     self.wifi_data_received = self.wifi_data_received.decode('utf-8')
    #                     if "wifi" in self.wifi_data_received:
    #                         # Define the type of protocol
    #                         self.type_protocol = "wifi"

    #                         # Answer the Micro, the data was received
    #                         time.sleep(0.3)
    #                         self.timer = self.after(500, self.first_wifi_confirmation)

    #                         #self.send_via_wifi
    #                         self.after_cancel(self.timer_check_connection)

    #                         #Set the Window 1 to start calibration
    #                         self.set_window1()

    #                         return
    #             except:
    #                 pass

    #     print("No Connection yet...")
    #             #time.sleep(0.5) 


    #     self.timer_check_connection = self.after(self.time_check, self.check_type_of_communication()) 

    def get_microcontroller_port(self):
        ports = serial.tools.list_ports.comports()
        microcontroller_port = None
        
        for port in ports:
            if "USB" in port.description:  # Modify this condition to match your microcontroller's description
                microcontroller_port = port.device
                break
        
        return microcontroller_port
           
    def set_window1(self):
        # =======================================================
        # ====================>  Label 1  <======================
        # =======================================================  
        self.label1_frame.grid_rowconfigure(1, weight=2)        
        self.label1.configure(text="Calibration Process - First Step")
        self.label1.grid(row=1, column=0, padx=20, pady=(20, 10))

        # =======================================================
        # ==========>  Frame 2 - First Instruction  <============
        # =======================================================  
        self.textbox = customtkinter.CTkTextbox(self.frame2, width=250, height=300)
        self.textbox.grid(row=0, column=0, padx=(0, 0), pady=(10, 10), sticky="nsew")
        self.textbox.insert("0.0","Get the MINIMAL Contraction\n\n" + "1 - Fix the bracelet in the middle of the \nupper-arm (biceps).\n\n" + "2 - Verify that the bracelet is not so tight!\n\n" + "3 - Place the arm in a vertical position,\n relaxing the biceps.\n\n" + "4 - Press the button Get.\n\n" + "5 - Wait the process")
        self.textbox.configure(font= self.my_font)

        # =======================================================
        # =================>  INSTALL BUTTON  <==================
        # =======================================================        
        self.main_button_1 = customtkinter.CTkButton(master=self, fg_color="transparent", 
                                                     border_width=3, text_color=("gray10", "#DCE4EE"), 
                                                     text="Get", font=self.my_font,
                                                     command=self.get_button1)        
        self.main_button_1.grid(row=3, column=1, padx=(10, 10), pady=(10, 10), sticky="e")

        # self.mensagem = self.ser.readline()
        # self.mensagem = self.mensagem.decode('utf-8')
        # print(self.mensagem)

    def set_window2(self):

        #print("Lets set the window 2")
        self.progressbar_1.destroy()
        # Text label 2
        self.label1.configure(text="Calibration Process - Second Step")

        # =======================================================
        # ==========>  Frame 2 - First Instruction  <============
        # =======================================================  
        self.textbox.delete("1.0", "end")
        self.textbox.insert("0.0", "Get the MAXIMUM Contraction\n\n" + "1 - Still with the bracelet in the middle \nof the upper-arm (biceps).\n\n" + "2 - Contract the biceps as much as \npossible!\n\n" + "3 - Press the button Get.\n\n" + "4 - Wait the process")
        #self.textbox.configure(font= self.my_font)

        # =======================================================
        # =================>  INSTALL BUTTON  <==================
        # =======================================================        
        self.main_button_1 = customtkinter.CTkButton(master=self, fg_color="transparent", 
                                                     border_width=3, text_color=("gray10", "#DCE4EE"), 
                                                     text="Get", font=self.my_font,
                                                     command=self.get_button2)        
        self.main_button_1.grid(row=3, column=1, padx=(10, 10), pady=(10, 10), sticky="e")

        # Confirmation
        # self.confirmation_txt = self.ser.readline()
        # self.confirmation_txt = self.confirmation_txt.decode('utf-8')

        # print(self.confirmation_txt)

    def set_window3(self):

        # Remove the loading bar    
        self.progressbar_1.destroy()

        # Verify if the (max - min) = 0 to avoid zero division!
        self.frame2.destroy()
        self.label1.configure(text="Calibration Done!", 
                                font=customtkinter.CTkFont(size=30, weight="bold"))

        # Close the Window Automatically
        self.main_button_1 = customtkinter.CTkButton(master=self, fg_color="transparent", 
                                                     border_width=3, text_color=("gray10", "#DCE4EE"), 
                                                     text="Finish", font=self.my_font,
                                                     command=self.close_setup)        
        self.main_button_1.grid(row=3, column=1, padx=(10, 10), pady=(10, 10), sticky="e")

    def first_wifi_confirmation(self):
        self.send_via_wifi()
        self.after_cancel(self.timer)

    def send_via_wifi(self):
        #self.gotit_message = "got_it"
        self.s.sendto(self.gotit_message.encode(), self.client_address)

    def send_via_serial(self):
        # Check if the microcontroller still connected
        #self.microcontroller_port = self.get_microcontroller_port()
        if (self.microcontroller_port == None):
            self.label1.configure(text="ERROR: probably the microcontroller \nwas disconnected!")
        else:                        
            try:    
                # =======================================> Send Got it
                print("Sending message via USB.")
                self.ser.write(self.gotit_message.encode())
                time.sleep(0.3)
                
                # Confirmation
                #print("I wrote")
                self.confirmation_txt = self.ser.readline()
                self.confirmation_txt = self.confirmation_txt.decode('utf-8')
                #print("I read")
                print(f"The confirmation: {self.confirmation_txt}")
                return
            except:
                self.label1.configure(text="ERROR: It looks like the microcontroller connected \n to another program, perhaps the Arduino IDE. \nPlease close this program before to continue!", 
                                      font=customtkinter.CTkFont(size=18, weight="bold"))

    def wait_calibration2(self):            
        try:
            if (self.type_protocol == "serial"):                
                
                # =======================================> Read the confirmation              
                #self.ser.write(self.nothing_message.encode())
                self.confirmation_txt = self.ser.readline()
                self.confirmation_txt = self.confirmation_txt.decode('utf-8')
                #print(f"I am receiving: {self.confirmation_txt}")

                if "got_min" in self.confirmation_txt:
                    print("Still here")                                                        

                elif "got_max" in self.confirmation_txt:
                    print("Maximum calibrated!")
                    #self.send_via_serial()
                    self.set_window3()
                    self.timer_wait = threading.Timer(self.period, self.wait_calibration2)
                    self.timer_wait.cancel()
                    return 
                else:
                    self.timer_wait = threading.Timer(self.period, self.wait_calibration2)
                    self.timer_wait.start()
                            

            elif (self.type_protocol == "wifi"):
                # Try to get data via wifi.
                for i in range(2):
                    self.wifi_data_received, self.client_address = self.s.recvfrom(self.BUFFER_SIZE)
                    if self.wifi_data_received:
                        self.wifi_data_received = self.wifi_data_received.decode('utf-8')
                        if "got_min" in self.wifi_data_received:
                            print("Still here")
                        elif "got_max" in self.wifi_data_received:
                            print("Maximum calibrated!")
                            #self.send_via_wifi()
                            self.set_window3()
                            self.timer_wait = threading.Timer(self.period, self.wait_calibration)
                            self.timer_wait.cancel()
                            return 
                        else:
                            self.timer_wait = threading.Timer(self.period, self.wait_calibration)
                            self.timer_wait.start()
        except:
            pass

    def wait_calibration(self):        
        #print("Chamei pela primeira vez!")        
        try:
            if (self.type_protocol == "serial"):                
                #print("Waiting1")
                
                # =======================================> Read the confirmation              
                self.ser.write(self.nothing_message.encode())
                self.confirmation_txt = self.ser.readline()
                self.confirmation_txt = self.confirmation_txt.decode('utf-8')
                #print(self.confirmation_txt)   
                # Confirmation
                # try:
                # except serial.serialutil.SerialTimeoutException:
                #     print("Not receiving data...")
                #     time.sleep(0.5)
                #     #self.ser.write(self.nothing_message.encode())
                #     self.confirmation_txt = self.ser.readline()
                #     self.confirmation_txt = self.confirmation_txt.decode('utf-8')
                #     print(self.confirmation_txt)
                #print("Waiting2")
                # The calibration ended
                if "got_min" in self.confirmation_txt:
                    print("Minimum calibrated!")
                    self.set_window2()
                    self.timer_wait = threading.Timer(self.period, self.wait_calibration)
                    self.timer_wait.cancel()
                    return                                                           

                elif "got_max" in self.confirmation_txt:
                    print("Maximum calibrated!")
                    self.send_via_serial()
                    self.set_window3()
                    self.timer_wait = threading.Timer(self.period, self.wait_calibration)
                    self.timer_wait.cancel()
                    return 
                else:
                    #self.timer_wait = self.after(self.period, self.wait_calibration())   
                    #print("schedule again")
                    self.timer_wait = threading.Timer(self.period, self.wait_calibration)
                    self.timer_wait.start()
                            

            elif (self.type_protocol == "wifi"):

                # Try to get data via wifi.
                self.wifi_data_received, self.client_address = self.s.recvfrom(self.BUFFER_SIZE)
                if self.wifi_data_received:
                    self.wifi_data_received = self.wifi_data_received.decode('utf-8')
                    if "got_min" in self.wifi_data_received:
                        print("Minimum calibrated!")
                        self.set_window2()
                        self.timer_wait = threading.Timer(self.period, self.wait_calibration)
                        self.timer_wait.cancel()
                        return  
                    elif "got_max" in self.wifi_data_received:
                        print("Maximum calibrated!")
                        self.send_via_wifi()
                        self.set_window3()
                        self.timer_wait = threading.Timer(self.period, self.wait_calibration)
                        self.timer_wait.cancel()
                        return 
                    else:
                        self.timer_wait = threading.Timer(self.period, self.wait_calibration)
                        self.timer_wait.start()
        except:
            pass     

    # =======================================================
    # ==================>  Event Functions  <================
    # =======================================================
    def close_setup(self):
        if (self.type_protocol == "serial"):
            self.send_via_serial()
        else:
            self.send_via_wifi()

        self.destroy()    
    def change_appearance_mode_event(self, new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)
    def change_scaling_event(self, new_scaling: str):
        new_scaling_float = int(new_scaling.replace("%", "")) / 100
        customtkinter.set_widget_scaling(new_scaling_float)

    def wifi_option(self):
        self.usb_button.destroy()
        try:
            for i in range(2):
                print("Trying to connect via wifi.")
                self.wifi_data_received, self.client_address = self.s.recvfrom(self.BUFFER_SIZE)
                if self.wifi_data_received: 
                    
                    self.wifi_data_received = self.wifi_data_received.decode('utf-8')
                    if "wifi" in self.wifi_data_received:

                        # Define the type of protocol
                        self.type_protocol = "wifi"                        

                        # Answer the Micro, the data was received
                        time.sleep(0.3)
                        self.timer = self.after(500, self.first_wifi_confirmation)

                        # Modify layout                      
                        self.set_window1()
                        break
        except:
            pass

    def usb_option(self):
        self.usb_button.destroy()  
        self.main_button_1.destroy()      
        self.microcontroller_port = self.get_microcontroller_port()
        self.timer_USB_info = self.after(self.period, self.read_USB_info)    
  
    def get_button1(self):  

        #print("Button clicked")

        self.main_button_1.destroy()
        # =================> Wait the Calibration process be done
        self.progressbar_1 = customtkinter.CTkProgressBar(self.frame2)
        self.progressbar_1.grid(row=3, column=0, padx=(10, 10), pady=(10, 10), sticky="ew")
        self.progressbar_1.configure(mode="indeterminnate")
        self.progressbar_1.start()

        self.period = 0.001                
        if (self.type_protocol == "serial"): 
            self.send_via_serial() # The confirmation of the clicking                
            self.confirmation_txt = " "            
            self.timer_wait = threading.Timer(self.period, self.wait_calibration)
            self.timer_wait.start()
            #self.timer_wait = self.after(self.period, self.wait_calibration())                        

        elif (self.type_protocol == "wifi"):
            #self.wifi_data_received, self.client_address = self.s.recvfrom(self.BUFFER_SIZE)
            self.send_via_wifi() 
            self.wifi_data_received = " "           
            self.timer_wait = threading.Timer(self.period, self.wait_calibration)
            self.timer_wait.start()
        else:
            self.label1.configure(text="Error! Plase restart the calibration program!")

    def get_button2(self):     

        #print("pressed second button") 
        self.main_button_1.destroy()
        # =================> Wait the Calibration process be done
        self.progressbar_1 = customtkinter.CTkProgressBar(self.frame2)
        self.progressbar_1.grid(row=3, column=0, padx=(10, 10), pady=(10, 10), sticky="ew")
        self.progressbar_1.configure(mode="indeterminnate")
        self.progressbar_1.start()

        self.period = 0.001
        if (self.type_protocol == "serial"): 
            self.send_via_serial()                   
            self.confirmation_txt = " "          
            self.timer_wait = threading.Timer(self.period, self.wait_calibration2)
            self.timer_wait.start()                          

        elif (self.type_protocol == "wifi"):
            #self.send_via_wifi() 
            self.timer = self.after(500, self.first_wifi_confirmation)
            self.wifi_data_received = " "           
            self.timer_wait = threading.Timer(self.period, self.wait_calibration2)
            self.timer_wait.start()
        else:
            self.label1.configure(text="Error! Plase restart the calibration program!")

#==================================================================================================
class Bracelet_Publisher(Node):
    
    def __init__(self):
        super().__init__('minimal_publisher')

        #self.get_logger().info(f' IT IS RUNNING THE READ NODE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')

        # ================================ CALIBRATION PART ==================================
        #self.calibration_app.mainloop()
        self.calibration_app = Calibration()
        self.calibration_app.mainloop()

        # ================================ PUBLISHING PART ==================================

        # Variables
        # The callback function to get the data
        if self.calibration_app.type_protocol == "serial":
            #period_getdata = 0.001
            period_getdata = 0.000666
            self.ser = self.calibration_app.ser
        else:
            period_getdata = 0.003
            self.s = self.calibration_app.s

        self.tp_check = self.calibration_app.type_protocol


        # The publisher
        self.publisher_ = self.create_publisher(Float64, '/fsr_topic', 1)
        
        self.fsr = 0.0
        self.timer_getdata = self.create_timer(period_getdata, self.get_data)

    def get_data(self):
        try:
            msg = Float64()

            # Check what is the protocol type
            if self.tp_check == "wifi":
                
                # Receive the Wifi Data
                #self.fsr, _ = self.calibration_app.s.recvfrom(self.calibration_app.BUFFER_SIZE) 
                self.fsr, _ = self.s.recvfrom(self.calibration_app.BUFFER_SIZE)              

                if self.fsr:
                    # Decode
                    self.fsr = self.fsr.decode('utf-8')

                    #self.get_logger.info(self.fsr)

                    if self.fsr == "wifi":                    
                        pass # The buffer is initially full of wifi message                        
                    else:
                        # Get sensors data
                        msg.data = float(self.fsr)
                        self.publisher_.publish(msg)

            else: # Serial comunication                
                self.fsr = self.ser.readline()
                msg.data = float(self.fsr.decode('utf-8'))
                self.publisher_.publish(msg)
            
            #self.get_logger.info(self.fsr)
        except:
            pass

def main(args=None):
    
    # ==================================> ROS Publisher 
    rclpy.init(args=args)
    minimal_publisher = Bracelet_Publisher()
    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

    # Close connection =============== DO NOT PUT THIS BEFORE THE ROS PUBLISHER, JUST IN THE END OF CODE
    minimal_publisher.calibration_app.s.close()

if __name__ == '__main__':
    main()   
