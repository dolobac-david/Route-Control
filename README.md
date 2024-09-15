This thesis sets itself the task to detect geometric changes, located on a previously
known route, which is considered as a template or pattern. Geometric changes represent
rigid transformations of objects in a scene, specifically their rotation, displacement and
their combinations. Another type of change is the addition of new or removal of original
objects. The original route is captured with a monocular camera and by using methods
of photogrammetry, a 3D model of a scene in a point representation is reconstructed
from the video sequence. In the same manner, a 3D model of the scene containing
changes is created, but this model captures only the local section of the original route.
This local point cloud is registered in the 3D model of the original route, so that the
3D models can be directly compared with each other. The investigated space is divided
into cubic voxels of the same size which are successively traversed and the degree of
similarity of local surfaces is investigated for each of them. The solution results in 3D
points from both clouds marked as changed or unchanged. The mentioned methods are
tested on real data in scenarios with different geometric modifications of the scene and
the evaluation of change detection is implemented as in the case of a binary classifier.
