import tkinter as tk       
import threading

class Application(tk.Frame):              
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)   
        self.grid()                       
        self.createWidgets()

    def printHello(self):
        print("Hello")

    def createWidgets(self):
        self.quitButton = tk.Button(self, text='Quit',
            command=self.quit) # exits background (gui) thread
        self.quitButton.grid(row=1,column=0)    
        self.printButton = tk.Button(self, text='Print',command=lambda: self.printHello())         
        self.printButton.grid(row=1,column=1) 

def runtk():  # runs in background thread
    app = Application()                        
    app.master.title('Sample application')     
    app.mainloop()
    
thd = threading.Thread(target=runtk)   # gui thread
thd.daemon = True  # background thread will exit if main thread exits
thd.start()  # start tk loop

while True:  # run in main thread
   x = input("Enter a value or Q to quit: ")
   if x.lower() == 'q':
      exit()
   print('You entered', x)