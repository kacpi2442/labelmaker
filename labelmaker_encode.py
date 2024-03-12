import ptcbp
from PIL import Image, ImageOps
from io import BytesIO

def encode_raster_transfer(data, nocomp=False):
    """ Encode 1 bit per pixel image data for transfer over serial to the printer """
    # Send in chunks of 1 line (128px @ 1bpp = 16 bytes)
    # This mirrors the official app from Brother. Other values haven't been tested.
    chunk_size = 16
    zero_line = bytearray(b'\x00' * chunk_size)

    for i in range(0, len(data), chunk_size):
        chunk = data[i : i + chunk_size]
        if chunk == zero_line:
            yield ptcbp.serialize_control('zerofill')
        else:
            yield ptcbp.serialize_data(chunk, 'none' if nocomp else 'rle')

def read_png(path, transform=True, padding=True, dither=True):
    """ Read a image and convert to 1bpp raw data

    This should work with any 8 bit PNG. To ensure compatibility, the image can
    be processed with Imagemagick first using the -monochrome flag.
    """
    image = Image.open(path)
    tmp = image.convert('1', dither=Image.FLOYDSTEINBERG if dither else Image.NONE)
    tmp = ImageOps.invert(tmp.convert('L')).convert('1')
    if transform:
        tmp = tmp.rotate(-90, expand=True)
        tmp = ImageOps.mirror(tmp)
    if padding:
        w, h = tmp.size
        padded = Image.new('1', (128, h))
        x, y = (128-w)//2, 0
        nw, nh = x+w, y+h
        padded.paste(tmp, (x, y, nw, nh))
        tmp = padded
    return tmp.tobytes()
