
Pallets - v2 2024-11-11 12:48pm
==============================

This dataset was exported via roboflow.com on November 11, 2024 at 12:50 PM GMT

Roboflow is an end-to-end computer vision platform that helps you
* collaborate with your team on computer vision projects
* collect & organize images
* understand and search unstructured image data
* annotate, and create datasets
* export, train, and deploy computer vision models
* use active learning to improve your dataset over time

For state of the art Computer Vision training notebooks you can use with this dataset,
visit https://github.com/roboflow/notebooks

To find over 100k other datasets and pre-trained models, visit https://universe.roboflow.com

The dataset includes 959 images.
Pallets-ground are annotated in YOLOv11 format.

The following pre-processing was applied to each image:
* Resize to 640x640 (Stretch)
* Auto-contrast via histogram equalization

The following augmentation was applied to create 3 versions of each source image:
* 50% probability of horizontal flip
* Random rotation of between -15 and +15 degrees
* Random brigthness adjustment of between -24 and +24 percent
* Random exposure adjustment of between -10 and +10 percent
* Salt and pepper noise was applied to 0.18 percent of pixels


