import numpy as np
from matplotlib import pyplot
import pandas

variable = 5

def function (argument):
    internalvariable = 5
    result = argument * internalvariable
    return result

y = function(10)
print(y)

class Dog:

    def __init__(self, name,age):
        self.name = name
        self.age = age
    def bark(self):
        print(self.name + " says woof!")
    def speech(self):
        print("Hello, my name is " + self.name + " and I am " + self.age + " years old!")
my_dog = Dog("Buddy","6")
my_dog.bark()
my_otherone = Dog("Enzo","8")
my_otherone.speech()