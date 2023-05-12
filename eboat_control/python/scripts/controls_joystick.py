import pygame
import time
import wx
import rospy
import numpy as np
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Point
from threading import Thread

class Sailor(wx.Frame):
    def __init__(self):
        super().__init__(parent=None, title='Sailor', size=(450, 450))

        pnl = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)
        hsizer = wx.BoxSizer()
        vsizer = wx.BoxSizer(wx.VERTICAL)

        trueWindSTR = wx.StaticText(pnl, label="True Wind Vector (x,y,z): ")
        title1 = wx.StaticText(pnl, label="Rudder")
        title2 = wx.StaticText(pnl, label="Sail")
        # title3 = wx.StaticText(pnl, label="Propultion")
        self.trueWindVec = wx.TextCtrl(pnl,
                                       value="(0,0,0)",
                                       style=wx.TE_PROCESS_ENTER)
        self.rudder = wx.Slider(pnl,
                                value=0,
                                minValue=-60,
                                maxValue=60,
                                style=wx.SL_HORIZONTAL | wx.SL_VALUE_LABEL)
        self.sail = wx.Slider(pnl,
                              value=0,
                              minValue=0,
                              maxValue=90,
                              style=wx.SL_HORIZONTAL | wx.SL_VALUE_LABEL)
        self.prop = wx.RadioBox(pnl,
                                label="Propultion engine",
                                choices=["-5", "-4", "-3", "-2", "-1", "0", "1", "2", "3", "4", "5"])
        hsizer.Add(trueWindSTR, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALIGN_LEFT | wx.LEFT, 5)
        hsizer.Add(self.trueWindVec, 0, wx.ALL | wx.LEFT, 5)
        vsizer.Add(hsizer)
        vsizer.AddSpacer(30)
        vsizer.Add(title1, 0, wx.ALIGN_BOTTOM | wx.ALIGN_LEFT | wx.LEFT, 5)
        vsizer.Add(self.rudder, 0, wx.ALL | wx.EXPAND, 5)
        vsizer.AddSpacer(30)
        vsizer.Add(title2, 0, wx.ALIGN_BOTTOM | wx.ALIGN_LEFT | wx.LEFT, 5)
        vsizer.Add(self.sail, 0, wx.ALL | wx.EXPAND, 5)
        vsizer.AddSpacer(30)
        vsizer.Add(self.prop, 0, wx.ALL | wx.LEFT, 10)

        self.prop.SetSelection(5)  # --> Set initial position

        main_sizer.Add(vsizer, 0, wx.EXPAND, 10)

        self.trueWindVec.Bind(wx.EVT_TEXT_ENTER, self.SetTrueWind)
        self.sail.Bind(wx.EVT_SLIDER, self.SailHandler)
        self.rudder.Bind(wx.EVT_SLIDER, self.RudderHandler)
        self.prop.Bind(wx.EVT_RADIOBOX, self.PropHandler)
        pnl.SetSizer(main_sizer)

        self.Show()

        rospy.init_node('Control_Interface', anonymous=True)
        self.bang_pub = rospy.Publisher('/eboat/control_interface/sail', Float32, queue_size=91)
        self.rang_pub = rospy.Publisher('/eboat/control_interface/rudder', Float32, queue_size=121)
        self.pvel_pub = rospy.Publisher('/eboat/control_interface/propulsion', Int16, queue_size=11)
        self.wind_pub = rospy.Publisher('/eboat/atmosferic_control/wind', Point, queue_size=5)
        self.reward = 0

        # Inicializa o Pygame e o joystick
        pygame.init()
        pygame.joystick.init()

        num_joysticks = pygame.joystick.get_count()
        if num_joysticks == 0:
            print("Nenhum joystick conectado.")
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.joystick_name = self.joystick.get_name()
        self.joystick_id = self.joystick.get_id()

        print(f"Joystick {self.joystick_name} (Entrada: {self.joystick_id})")

        # Configura o intervalo de atualização dos eventos do joystick
        pygame.event.set_allowed(None)
        pygame.event.set_allowed(pygame.JOYAXISMOTION)
        pygame.event.set_allowed(pygame.JOYBUTTONDOWN)
        pygame.event.set_allowed(pygame.JOYBUTTONUP)
        pygame.event.set_allowed(pygame.JOYHATMOTION)

        # Cria uma thread separada para os eventos do joystick
        self.joystick_thread = Thread(target=self.joystick_events)
        self.joystick_thread.daemon = True
        self.joystick_thread.start()

    def joystick_events(self):
        try:
            while True:
                pygame.event.pump()

                # Obtém os valores dos eixos
                for i in range(self.joystick.get_numaxes()):
                    axis = self.joystick.get_axis(i)
                    print(f"Eixo {i}: {axis:.2f}")

                    if i == 1:  # Eixo 1 controla a vela
                        if axis > 0.3:
                            selection = self.sail.GetValue() + 1
                            if selection >= 90:
                                selection = 90
                            self.sail.SetValue(selection)
                        elif axis < -0.3:
                            selection = self.sail.GetValue() - 1
                            if selection <= -90:
                                selection = -90
                            self.sail.SetValue(selection)
                    self.SailHandler(None)
                    
                    if i == 3:  # Eixo 3 controla 0 leme
                        if axis > 0.3:
                            selection = self.rudder.GetValue() + 1
                            if selection >= 60:
                                selection = 60
                            self.rudder.SetValue(selection)
                        elif axis < -0.3:
                            selection = self.rudder.GetValue() - 1
                            if selection <= -60:
                                selection = -60
                            self.rudder.SetValue(selection)
                    self.RudderHandler(None)

                # Obtém os valores dos botões
                for i in range(self.joystick.get_numbuttons()):
                    button = self.joystick.get_button(i)
                    print(f"Botão {i}: {button}")

                # Obtém os valores dos hats (direcionais)
                for i in range(self.joystick.get_numhats()):
                    hat = self.joystick.get_hat(i)
                    print(f"Hat {i}: {hat}")

                    if i == 0 and hat == (0, -1):  # Hat 0, posição (0, -1) liga o motor em decrementos de -1
                        selection = self.prop.GetSelection() - 1
                        if selection <= -5:
                            selection = -5
                        self.prop.SetSelection(selection)
                        self.PropHandler(None)
                    elif i == 0 and hat == (0, 1):  # Hat 0, posição (0, 1) liga o motor em incrementos de +1
                        selection = self.prop.GetSelection() + 1
                        if selection >= +10:
                            selection = 10
                        self.prop.SetSelection(selection)
                        self.PropHandler(None)

                print("--------------------")

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("Programa encerrado pelo usuário.")

        finally:
            self.joystick.quit()
            pygame.quit()

    def SetTrueWind(self, event):
        text = self.trueWindVec.GetLineText(0).translate({ord(i): None for i in '()'})
        wvec = np.array(text.split(","), dtype=np.float)
        if not rospy.is_shutdown():
            self.wind_pub.publish(Point(wvec[0], wvec[1], wvec[2]))

    def SailHandler(self, event):
        if not rospy.is_shutdown():
            self.bang_pub.publish(self.sail.GetValue())

    def RudderHandler(self, event):
        if not rospy.is_shutdown():
            self.rang_pub.publish(-self.rudder.GetValue())

    def PropHandler(self, event):
        if not rospy.is_shutdown():
            self.pvel_pub.publish(int(self.prop.GetString(self.prop.GetSelection())))

if __name__ == '__main__':
    app = wx.App(False)
    frame = Sailor()
    app.MainLoop()
