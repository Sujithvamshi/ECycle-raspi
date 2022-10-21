import time, threading

StartTime=time.time()
def action() :
    print('action ! -> time : {:.1f}s'.format(time.time()-StartTime))
def action2(inter):
    s = input("enter")
    if s=="s":
        inter.cancel
    
class setInterval :
    def __init__(self,interval,action) :
        self.interval=interval
        self.action=action
        self.stopEvent=threading.Event()
        thread=threading.Thread(target=self.__setInterval)
        thread.start()

    def __setInterval(self) :
        nextTime=time.time()+self.interval
        while not self.stopEvent.wait(nextTime-time.time()) :
            nextTime+=self.interval
            self.action()

    def cancel(self) :
        self.stopEvent.set()
inter = setInterval(0.6,action)
inter2 = setInterval(0.3,action2(inter))