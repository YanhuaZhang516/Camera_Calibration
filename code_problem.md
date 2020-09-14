# some problem I have met when I code:

## problem 1:

```
ValueError: The truth value of an array with more than one element is ambiguous. Use a.any() or a.all()

```
我本来想用来判断算出来的数据是否为空的，当程序运行的时候，如果算不出来，令其为0。实际上在numpy数组中`a==0`, 有两种情况: 是否存在一个数为0; 或者所有数组都为0.
判断是否存在 0，用 `a.any()`; 是否全为0, 用`a.all()`。

所以两种情况都不是我想要的，应该另数组为`a=None`

## problem 2:
```
Multiple realsense udev-rules were found:
1:/etc/udev/rules.d/99-realsense-libusb.rules
2: /lib/udev/rules.d/60-librealsense2-udev-rules.rules
Make sure to remove redundancies!

```
I ran the command:
```
sudo cp ~/.99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && udevadm trigger
```
and removed the duplicate:
```
sudo rm /lib/udev/rules.d/60-librealsense2-udev-rules.rules
```
the problem is solved and I don't get the warning anymore.
