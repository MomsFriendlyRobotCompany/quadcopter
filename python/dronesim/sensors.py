# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2022 Kevin Walchko
# see LICENSE for full details
##############################################
import numpy as np
from dataclasses import dataclass
from dataclasses import field


# @dataclass
class SensorNoise:
    # mu: float
    # sigma: float
    # noise_size: int = 1000
    def __init__(self, mu, sigma, dim, noise_size=10000):
        """
        mu: mean, can be a float or list of size noise_size
        sigma: standard dev, can be float or list of size noise_size
        dim: dimension, how many readings per sensor, my Accels have 3, (x,y,z)
        noise_size: how many random samples to get
        """
        self.mu = mu
        self.sigma = sigma
        self.noise_size = noise_size
        self.noise = np.random.normal(mu, sigma, (noise_size, dim))
        self.cnt = 0

    def read(self, s):
        self.cnt = (self.cnt + 1) % self.noise_size
        return self.noise[self.cnt, :] + s


# # @dataclass
# class Accels(SensorNoise):
#     # bias: float
#     # std: float
#     # noise_size: int = 1000 #field(init=False, default=1000)
#     # cnt: int = 0 #field(init=False, default=0)
#     # n: np.ndarray #= self.noise(self.noise_size, 3)

#     # noise_size = 1000
#     def __init__(self, bias, sigma):
#         super().__init__(bias, sigma)
#         self.n = self.noise(self.noise_size, 3)
#         self.cnt = 0

#     def read(self, a):
#         self.cnt = (self.cnt + 1) % self.noise_size
#         return self.n[self.cnt, :] + a


# class Gyros(SensorNoise):
#     # noise_size = 1000
#     def __init__(self, bias, sigma):
#         super().__init__(bias, sigma)
#         self.n = self.noise(self.noise_size, 3)
#         self.cnt = 0

#     def read(self, s):
#         self.cnt = (self.cnt + 1) % self.noise_size
#         return self.n[self.cnt, :] + s


# class Mags(SensorNoise):
#     # noise_size = 1000
#     def __init__(self, bias, sigma):
#         super().__init__(bias, sigma)
#         self.n = self.noise(self.noise_size, 3)
#         self.cnt = 0

#     def read(self, s):
#         self.cnt = (self.cnt + 1) % self.noise_size
#         return self.n[self.cnt, :]



# class Barometer(SensorNoise):
#     # noise_size = 1000
#     def __init__(self, bias, sigma):
#         super().__init__(bias, sigma)
#         self.n = self.noise(self.noise_size, 2)
#         self.cnt = 0

#     def read(self):
#         self.cnt = (self.cnt + 1) % self.noise_size
#         return self.n[self.cnt, :]