# Controlling the Brother P-Touch Cube label maker from a computer

The Brother PTP300BT label maker is intended to be controlled using the official Brother P-Touch Design & Print iOS/Android app. The app has arbitrary limits on what you can print (1 text object and up to 3 preset icons), so I thought it would be a fun challenge to reverse engineer the protocol to print whatever I wanted.

Python code at the bottom if you want to skip the fine details.

**This is a fork of the original gist that adds Python 3 support (3.6+) and have various adjustments. If you want information on the protocol details please refer to the original version**

## Python code

The code here is what I had at the point I got this working - it's a bit hacked together. It prints images, but the status messages printed aren't complete and the main script needs some tidying up. The printer sometimes goes to an error state after printing (haven't figured out why yet), which can be cleared by pressing the power button once.

This needs a few modules installed to run:

```
pybluez
pillow
packbits
```

Then it can be used as:

```sh
# Existing image (typical use case)
./labelmaker.py <bdaddr of your printer> -i horizontal-label-image.png

# Existing image formated to spec above (advanced)
# -r disables all built-in image pre-processing
./labelmaker.py <bdaddr of your printer> -i monochrome-128px-wide-image.png -r

# (If using option 2)
# Using imagemagick to get a usable input image from any horizontal oriented image
# -resize 128x can be used instead of -crop 128x as needed
# -rotate 90 can be removed if the image is portrait already
convert inputimage.png -monochrome -gravity center -crop 128x -rotate 90 -flop out.png
```