from svglib.svglib import svg2rlg
from reportlab.graphics import renderPM
import sys
import os

    # handler = rsvg.Handle(path+svgFileName)
    #     img = cairo.ImageSurface(cairo.FORMAT_ARGB32, handler.props.width,handler.props.height)
    #     handler.render_cairo(cairo.Context(img))
    #     img.write_to_png( output_dir + svgFileName.replace(".svg",".png"))

def convertSvgFolder(path, outputDir):
    fileList = os.listdir(path)
    for x in fileList:
    	convertSvg2Png(path, x, outputDir)
    print (len(fileList), " file(s) written to folder ", outputDir)

def convertSvg2Png(path, svgFileName,output_dir):
	drawing = svg2rlg(path+svgFileName)
	renderPM.drawToFile(drawing, output_dir + svgFileName.replace(".svg",".gif"), fmt="GIF")

def main(argv):
    usage = 'Usage: '+ argv[0] + " <Path to SVG>\n"
    usage = usage + "Converts all svg files to gif."
    if len(argv) == 1:
        print (usage)
    elif len(argv) == 2:
        convertSvgFolder(argv[1],argv[1])
    elif len(argv) == 3:
        convertSvgFolder(argv[1],argv[2])
    else:
        print (usage)

if __name__ == "__main__":
   main(sys.argv[:])



