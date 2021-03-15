from m5stack import *
from m5ui import *
from uiflow import *
from machine import UART
import time

# CONST
## person icon
NUMOFPERSON = 5
SPACE = 5
WINDOW_W = 320
WINDOW_H = 240
BACK_COLOR = 0x222222

## message points
COUNT_X, COUNT_Y = (55, 0)
SLOG_X, SLOG_Y = (65, 220)
SERR_X, SERR_Y = (55, 20)
VLOG_X, VLOG_Y = (110, 40)

## UnitV
PACKET_DIV = "_"
VTX, VRX = (22, 21) #22,21

## Sig
STX, SRX = (17, 16) # 17,16
SIG_INTERVAL = 600 # 600
RECIVE_INTERVAL = 60
LOG_INTERVAL = 60

# person config
params = {"face_r":15, "height":50, 
          "width":60, "color":0xffffff,
}
class Person:
  def __init__(self, position, face_r=15, 
                height=50, width=40, color=0x222222):
    # criterion
    x0 = (WINDOW_W - NUMOFPERSON*width - (NUMOFPERSON-1)*SPACE) // 2
    y0 = (WINDOW_H - height) // 2 + 20
    
    # x
    left = x0 + (width+SPACE)*position
    center = left + (width // 2)
    
    # y
    nose = y0 + face_r
    waist = y0 + height
    
    body_h = height - face_r*2
    self.face = [center, nose, face_r]
    self.body = [center, waist, width//2, body_h]
    self.mask = [left, waist, width, body_h]
    self.area = [left, y0, width, height+body_h]
    self.color = color
    
  def hide(self):
    lcd.rect(self.area[0], self.area[1], self.area[2], self.area[3],
              color=BACK_COLOR, fillcolor=BACK_COLOR)
    
  def show(self):
    lcd.circle(self.face[0], self.face[1], self.face[2], 
                color=BACK_COLOR, fillcolor=self.color)
    lcd.ellipse(self.body[0], self.body[1], self.body[2], self.body[3],
                color=BACK_COLOR, fillcolor=self.color)
    lcd.rect(self.mask[0], self.mask[1], self.mask[2], self.mask[3], 
              color=BACK_COLOR, fillcolor=BACK_COLOR)


class Contoller:
  def __init__(self, num=NUMOFPERSON, **params,):
    self.num = num
    self.params = params
    self.Plist = None
    self.buffer = 0
    
  def init(self,):
    self.Plist = None

  def make_persons(self,):
    self.Plist = [Person(i, **self.params) for i in range(self.num)]
    
  def draw_persons(self, detects):
    if (self.buffer - detects) > 0:
      for i in range(detects, self.buffer):
        self.Plist[i].hide()
    else:
      for i in range(self.buffer, detects):
        self.Plist[i].show()
    self.buffer = detects
    

def uart_init():
  # Sigfox
  uart1 = UART(1, tx=STX, rx=SRX)
  uart1.init(9600, bits=8, parity=None, stop=1)
  
  # UnitV
  uart2 = UART(2, tx=VTX, rx=VRX)
  uart2.init(115200, bits=8, parity=None, stop=1)
  return uart1, uart2

def make_message(l, r, cumsum):
  #  L    R  CUMSUM 
  # 000  000  0000  00
  msg = l*16**9 + r*16**6 + cumsum*16**2
  msg = "{:012x}".format(msg)
  return msg

def recieve():
  string = uart1.read()
  try:
    lcd.print("Sigfox result: "+string.decode(), SLOG_X, SLOG_Y, 0xffffff)
  except:
    lcd.print("Sigfox result: "+string, SLOG_X, SLOG_Y, 0xff0000)
  wait_ms(100)
  return time.time()

def sendMessage(x):
  # upload frequency
  uart1.write('AT$IF=923200000'+"\r\n")
  wait_ms(100)
  recieve()
  
  # Set output power
  uart1.write('ATS302=15'+"\r\n")
  wait_ms(100)
  recieve()
  
  # main message
  uart1.write(str((str('AT$SF=') + str(x)))+"\r\n")
  wait_ms(100)
  return time.time()

  
#################################################
def main():
  # main
  ## init
  setScreenColor(BACK_COLOR)
  C = Contoller(num=NUMOFPERSON, **params)
  C.make_persons()
  uart1, uart2 = uart_init()
  
  ## set conditional variables
  V_check = 0
  send_time, recieve_time = (0,0)
  is_send, is_recieve = (True, False)
  base_l, base_r, pre_l, pre_r, left, right, left_per_T, right_per_T = (0,0,0,0,0,0,0,0)
  
  ## main loop
  while True:
    # start Sig timer
    if is_send:
      start = time.time()
      is_send = False
    
    l = uart2.any()
    if l > 0:
      # read
      buf = uart2.read().decode('utf-8')
      try:
        tmp_l, tmp_r, detects = buf.split(PACKET_DIV)
      except ValueError as e:
        continue
      
      # check counter reset
      tmp_l, tmp_r = (int(tmp_l), int(tmp_r))
      if pre_l > tmp_l:
        base_l = left
        base_r = right
      left = tmp_l + base_l
      right = tmp_r + base_r
      
      # display
      msg = str(right)+"<-  COUNTER  ->"+str(left)
      lcd.textClear(COUNT_X, COUNT_Y, msg)
      lcd.text(COUNT_X, COUNT_Y, msg)
      C.draw_persons(int(detects))
      
      # send Sig
      if time.time() - start > SIG_INTERVAL:
        msg = make_message(left-left_per_T, right-right_per_T, left+right)
        try:
          send_time = sendMessage(msg)
        except TypeError:
          lcd.print("Sigfox result: NG", SERR_X, SERR_Y, 0xff0000)
        else:
          lcd.textClear(SERR_X, SERR_Y, "Sigfox result: NG")
          is_recieve = True
        left_per_T = left
        right_per_T = right
        is_send = True
        
        
      # finalize
      lcd.textClear(VLOG_X, VLOG_Y, "NO SIGNAL")
      V_check = 0
      pre_l = tmp_l
      pre_r = tmp_r
      if (time.time() - send_time > RECIVE_INTERVAL) and is_recieve:
        try:
          recieve_time = recieve()
        except TypeError:
          lcd.print("Sigfox result: NG", SERR_X, SERR_Y, 0xff0000)
        else:
          lcd.textClear(SERR_X, SERR_Y, "Sigfox result: NG")
        is_recieve = False
      if time.time() - recieve_time > LOG_INTERVAL:
        lcd.textClear(SLOG_X, SLOG_Y, " "*30)
    else:
      V_check += 1
      if V_check > 10000:
        lcd.text(VLOG_X, VLOG_Y, "NO SIGNAL")
        V_check -= 1
        
main()
