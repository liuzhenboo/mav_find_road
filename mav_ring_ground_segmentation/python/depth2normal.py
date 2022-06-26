import cv2
img=cv2.imread('/home/lzb/Ground_Pointcloud_Extract/src/depth_extract/data/depth/test.png')
print(img.item(1,1,2))

