/* Commands for creating collision meshes for Unreal, 
* 
* UBX - Box Collider, using Optimal Bounding Box
* USP - Sphere Collider, using Bounding Volumes
* UCX - Convex Collider, using 3D Convex Hulls
* 
* See: https://docs.unrealengine.com/4.27/en-US/WorkingWithContent/Importing/FBX/StaticMeshes/
*/

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>

#include <CGAL/Real_timer.h>

#include <fstream>
#include <iostream>

#include <lxu_command.hpp>

#include <lxidef.h>
#include <lx_layer.hpp>
#include <lx_mesh.hpp>

#include <lxu_select.hpp>


namespace PMP = CGAL::Polygon_mesh_processing;

typedef CGAL::Exact_predicates_inexact_constructions_kernel    K;
typedef K::Point_3                                             Point;

typedef CGAL::Surface_mesh<Point>                              Surface_mesh;

//int main(int argc, char** argv)
//{
//    const std::string filename = (argc > 1) ? argv[1] : CGAL::data_file_path("meshes/pig.off");
//
//    Surface_mesh sm;
//    if (!PMP::IO::read_polygon_mesh(filename, sm) || sm.is_empty())
//    {
//        std::cerr << "Invalid input file." << std::endl;
//        return EXIT_FAILURE;
//    }
//
//    CGAL::Real_timer timer;
//    timer.start();
//
//    // Compute the extreme points of the mesh, and then a tightly fitted oriented bounding box
//    std::array<Point, 8> obb_points;
//    CGAL::oriented_bounding_box(sm, obb_points,
//        CGAL::parameters::use_convex_hull(true));
//
//    std::cout << "Elapsed time: " << timer.time() << std::endl;
//
//    // Make a mesh out of the oriented bounding box
//    Surface_mesh obb_sm;
//    CGAL::make_hexahedron(obb_points[0], obb_points[1], obb_points[2], obb_points[3],
//        obb_points[4], obb_points[5], obb_points[6], obb_points[7], obb_sm);
//    std::ofstream("obb.off") << obb_sm;
//
//    PMP::triangulate_faces(obb_sm);
//    std::cout << "Volume: " << PMP::volume(obb_sm) << std::endl;
//
//    return EXIT_SUCCESS;
//}

#define SRVNAME_COMMAND	"ubx.new"

using namespace lx_err;

class COptimalBoundingBox : public CLxBasicCommand {
public:
    COptimalBoundingBox();
    int basic_CmdFlags() LXx_OVERRIDE;
    void basic_Execute(unsigned flags);

private:
    // Services we will be using,
    CLxUser_SceneService scene_service;

    LXtItemType mesh_type;
    CLxUser_Item item;
};

COptimalBoundingBox::COptimalBoundingBox() {

}

int COptimalBoundingBox::basic_CmdFlags() {
    return LXfCMD_MODEL | LXfCMD_UNDO;
}

void COptimalBoundingBox::basic_Execute(unsigned flags) {
    // Get the currently active scene,
    CLxSceneSelection scene_selection;
    CLxUser_Scene scene;
    scene_selection.Get(scene);    

    // Get the LXtItemType for Mesh items,
    scene_service.ItemTypeLookup(LXsITYPE_MESH, &mesh_type);
    
    // If scene is not null add a new mesh item.
    if (scene.test()) {
        scene.ItemAdd(mesh_type, item);
    }
}

void initialize() {
    CLxGenericPolymorph* srv;

    srv = new CLxPolymorph<COptimalBoundingBox>;
    srv->AddInterface(new CLxIfc_Command<COptimalBoundingBox>);
    srv->AddInterface(new CLxIfc_Attributes<COptimalBoundingBox>);
    srv->AddInterface(new CLxIfc_AttributesUI<COptimalBoundingBox>);
    lx::AddServer(SRVNAME_COMMAND, srv);
}