# Rerun visualization for KITTI dataset

This is a repository for [Rerun]() visualization of the KITTI dataset.

Currently, only the raw-sync dataset is supported.

raw-extract dataset will be supported soon.

## Dependencies

- rerun
- pykitti
- numpy
- scipy

Use the command below 

```
pip3 install -r requirements.txt 
```

## How to run

Use the command below, when you have the folder structure '~/kitti/raw/2011_09_30/2011_09_30_drive_00XX_sync/'.

``` bash
python 3 --basedir ~/kitti/raw --date 2011_09_30 --drive 0020
```

## Output

![](output.gif)
