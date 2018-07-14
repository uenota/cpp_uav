# cpp_uav
Coverage path planning package for UAVs

## Nodes
- specify_rect.py  
Shows GUI to specify region for coverage path planner.

- torres_etal_2016  
Coverage path planner based on [1]

## Launch Files
- cpp_uav.launch  
Launches specify_rect.py and torres_etal_2016.

## Document
You can build documents by running 
```
doxygen Doxyfile
```
at the root of `cpp_uav` repository.
And the documents will be build in `doc` directory.

## Reference
[1] Marina Torres, David A. Pelta, José L. Verdegay, Juan C. Torres,  
    Coverage path planning with unmanned aerial vehicles for 3D terrain reconstruction,  
    In Expert Systems with Applications,  
    Volume 55, 2016, Pages 441-451, ISSN 0957-4174, (https://doi.org/10.1016/j.eswa.2016.02.007).
