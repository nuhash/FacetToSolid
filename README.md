#Pre-description
This is a software project that I worked on to complete my MEng in Mechanical Engineering.

There are several bugs in the code but they do not prevent the purpose of the project being fulfilled.

#Actual description
This project aimed to develop a program that could extract features from a facet model (as an STL file) and reconstruct them as surface of a Boundary Representation scheme model.

I have implemented 2 algorithms (Vertex-wise Normal Tensor Framework Method as HybridEdgewiseNormalTensorFrameworkMethod in the code; Edge-wise Normal Tensor Framework Method as EdgewiseMethod in the code).

I have tested the algorithms using some models that I had made and the results showed that not only were algorithms capable of extracting simple features but that the general principles could scale to extract more complex ones.

#Requirements
This has only been tested on one machine, I would the only specific hardware required is a modern GPU. Integrated GPUs should suffice. For CPU, my benchmarks did not go longer than a second with 2 year old enthusiast-grade CPU.

##Compiling
You need Eigen, wxWidgets and OpenCASCADE. I can't provide help for setting it up, I had a lot problems doing so and my solutions weren't that great. If you are familar with compiling C++ projects then you should be able to figure it out by looking at the Visual Studio 2015 project files.

Some changes need to be made to get it to compile for Linux and Unix, I believe the GUI will require some changes as the 3D view area is handled differently on non-Windows platforms. You may also have to replace some utility functions provided by Windows.
