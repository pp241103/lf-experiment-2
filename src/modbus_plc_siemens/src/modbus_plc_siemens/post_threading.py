import threading
import sys


class PostThread(threading.Thread):
    # Main thread functions
    
    def __init__(self, func):
        threading.Thread.__init__(self, )
        self.killed = False
        self.daemon = True
        self.func = func

    def execute(self, *args, **kwargs):
        self.kwargs = kwargs
        self.args = args
        self.start()
        return self

    def run(self):
        self.func(*self.args, **self.kwargs)

# ============================================

class PostKillingThread(PostThread):
    def __init__(self, func):
        PostThread.__init__(self, func)
        
    def start(self):
        self.__run_backup = self.run
        self.run = self.__run
        threading.Thread.start(self)

    def __run(self):
        sys.settrace(self.globaltrace)
        self.__run_backup()
        self.run = self.__run_backup

    def globaltrace(self, frame, event, arg):
        if event == 'call':
            return self.localtrace
        else:
            return None

    def localtrace(self, frame, event, arg):
        if self.killed:
            if event == 'line':
                raise SystemExit()
        return self.localtrace

    def kill(self):
        self.killed = True

# ============================================

class Post:
    """Class providing threaded calls for parent object' methods"""

    def __init__(self, parent):
        self.parent = parent

    def __getattr__(self, attr):
        """1. Finds the method asked for in parent object
           2. Encapsulates this method in thread' object
           3. Returns its execution function to caller"""
        try:
            func = getattr(self.parent, attr)
            post_thread = PostKillingThread(func)
            return post_thread.execute
        except:
            raise Exception(
                f"Error: Post call on {str(self.parent)} (method {attr} not found)")
    
    @staticmethod   
    def killall():
        try:
            for thread in threading.enumerate():
                thread.__class__ = PostKillingThread
                thread.kill()
        except: pass
