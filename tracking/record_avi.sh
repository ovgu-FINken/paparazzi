#!/bin/bash

mencoder tv:// -tv driver=v4l2:width=1280:height=720:outfmt=uyvy:device=/dev/video1:input=1:fps=25 -nosound -ffourcc DX50 -ovc lavc -lavcopts vcodec=mpeg4:mbd=2:turbo:vbitrate=1200:keyint=15 -o Aufnahme.avi