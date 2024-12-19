#!/usr/bin/env python3
#
# nonblockinginput.py
#
# Create a non-blocking input class. This allows checking whether
# an input line is available before trying to get() the line.
#
# nbi = NonBlockingInput(prompt="", silent=False)
# Initialize with a default prompt
#
# nbi.prompt(prompt) Print a prompt (use default if not given)
# nbi.ready() Return True if an input is waiting
# nbi.get() Return an input, or None if none ready
#

# Imports
import queue
import sys
import threading

#
# Internal Queue and Thread
#
# This creates a thread to monitor the input. As well as a
# thread-safe queue, so the input lines can be passed/seen outside
# the thread.
#
# Note the thread is marked as daemon, so it is automatically killed
# when the main thread exits. It is also created outside the class,
# so only one thread will exist. Should multiple objects be
# instantiated, they will "fight" for who gets the input.
#
inputlines = queue.Queue()

def readInputLines():
    while True:
        # Pass input lines from the standard input into the queue.
        inputlines.put(sys.stdin.readline().strip())

inputthread = threading.Thread(name="NBInput",target=readInputLines)
inputthread.daemon = True
inputthread.start()

#
# Non-Blocking Input Class
#
# This provides a convienent interface. Adding the __enter__() and
# __exit__() functions allows a "with NonBlockingInput() as nbi" usage.
#
class NonBlockingInput:
    def __init__(self, prompt="", silent=False):
        # Save the default prompt.
        self.defaultprompt = prompt
        # Also clear/flush the input line queue.
        while not inputlines.empty():
            inputlines.get()
        # And display the prompt (unless silent).
        if not silent:
            self.prompt()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass

    def prompt(self, prompt=None):
        if prompt is None:
            prompt = self.defaultprompt
        print(prompt, end="", flush=True)

    def ready(self):
        return not inputlines.empty()

    def get(self):
        try:
            return inputlines.get(block=False)
        except queue.Empty:
            return None
