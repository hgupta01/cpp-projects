#include <surf_descriptor.h>

BOOST_PYTHON_MODULE(surfmodule)
{
    Py_Initialize();
    class_<MyList>("MyList")
        .def(vector_indexing_suite<MyList>() );

    class_<SURFDescriptor>("SURFDescriptor", init<std::string, std::string>())
				.def("calculateMatchingPoints", &SURFDescriptor::calculateMatchingPoints)
        .def("getSrcPnts", &SURFDescriptor::getSrcPnts)
        .def("getTrgPnts", &SURFDescriptor::getTrgPnts)
        ;
}

// BOOST_PYTHON_MODULE(surfmodule)
// {
//     class_<MyList>("MyList")
//         .def(vector_indexing_suite<MyList>() );

//     class_<SURFDescriptor>("SURFDescriptor")
// 				.def("calculateMatchingPoints", &SURFDescriptor::calculateMatchingPoints)
// 				.def("setSrcImg", &SURFDescriptor::setSrcImg)
//         .def("setTrgImg", &SURFDescriptor::setTrgImg)
//         .def("getSrcPnts", &SURFDescriptor::getSrcPnts)
//         .def("getTrgPnts", &SURFDescriptor::getTrgPnts)
//         ;
// }