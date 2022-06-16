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
#include <lx_action.hpp>
#include <lx_layer.hpp>
#include <lx_mesh.hpp>
#include <lxu_vector.hpp>
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
    CLxUser_LayerService layer_service;

    CLxUser_Scene scene;  // will be set to the context of selected item, and used to add items to scene

    LXtItemType mesh_type;  // type, so we can tell the scene to add a "mesh",
    CLxUser_Item item;  // the item we will be creating, if all goes well
    CLxUser_Mesh mesh;
    CLxUser_Point point_accessor;  // Point accessor to read and write point data from mesh in Modo,
    CLxUser_Polygon polygon_accessor; // Polygon accessor to write polygons to meshe in Modo,
};

// We don't currently need to initialize any inputs so constructor is empty,
COptimalBoundingBox::COptimalBoundingBox() {}

// Notify Modo this command will change scene state and that we want to be able to undo it,
int COptimalBoundingBox::basic_CmdFlags() {
    return LXfCMD_MODEL | LXfCMD_UNDO;
}

void COptimalBoundingBox::basic_Execute(unsigned flags) {
    // Get the LXtItemType for Mesh items,
    scene_service.ItemTypeLookup(LXsITYPE_MESH, &mesh_type);

    // Get the primary selected mesh
    CLxUser_LayerScan primary_layer;
    check(layer_service.ScanAllocate(LXf_LAYERSCAN_PRIMARY, primary_layer));

    // If we had a mesh selected, keep going otherwise exit early here.
    unsigned int any_primary_layer;
    primary_layer.Count(&any_primary_layer);
    if (!any_primary_layer)
        return;

    // Get the first available mesh in layer scan,
    check(primary_layer.BaseMeshByIndex(0, mesh));
    check(primary_layer.ItemByIndex(0, item));
    item.GetContext(scene);  // Get the scene the item is in.

    // Store number of points so we can initialize a vector K::Point_3 of same size
    unsigned int num_points;
    mesh.PointCount(&num_points);
    std::vector<Point> input_points(num_points);

    // Iterate over all points in primary selected mesh, and populate vector of points,    
    point_accessor.fromMesh(mesh);
    LXtFVector position;
    for (unsigned i = 0; i < num_points; i++) {
        point_accessor.SelectByIndex(i);
        point_accessor.Pos(position);
        input_points[i] = Point(position[0], position[1], position[2]);
    }

    // Apply, clear and all that jazz for the layer scan,
    primary_layer.Apply();
    primary_layer.clear();
    primary_layer = NULL;


    // Run the CGAL method to get the Optimal Bounding Box for all the points,
    std::array<Point, 8> optimal_bounding_box_points;  // The eight points making up the bounding box will be stored to this array,
    CGAL::oriented_bounding_box(input_points, optimal_bounding_box_points,
        CGAL::parameters::use_convex_hull(true));


    // Exit if scene is not valid,
    if (!scene.test())
        return;

    scene.ItemAdd(mesh_type, item);

    LXtObjectID obj;
    // Now to create the new mesh data, we need a channel write object
    CLxUser_ChannelWrite channel_write;
    check(scene.Channels(LXs_ACTIONLAYER_EDIT, 0.0, (void**)&obj));
    check(channel_write.take(obj));
    unsigned channel_index;
    check(item.ChannelLookup(LXsICHAN_MESH_MESH, &channel_index));  // first getting the channel index for the mesh data,
    
    check(channel_write.ValueObj(item, channel_index, (void **) &obj));
    check(mesh.take(obj));

    point_accessor.fromMesh(mesh);  // set the point accessor to the new mesh

    LXtVector set_position;  // reading is done in floats while writing position is done using doubles for some reason.
    LXtPointID point_id;
    std::array<LXtPointID, 8> point_ids;
    for (unsigned index = 0; index < 8; index++) {

        set_position[0] = optimal_bounding_box_points[index].x();
        set_position[1] = optimal_bounding_box_points[index].y();
        set_position[2] = optimal_bounding_box_points[index].z();

        point_accessor.New(set_position, &point_id);
        point_ids[index] = point_id;
    }

    mesh.SetMeshEdits(LXf_MESHEDIT_GEOMETRY);
}

void initialize() {
    CLxGenericPolymorph* srv;

    srv = new CLxPolymorph<COptimalBoundingBox>;
    srv->AddInterface(new CLxIfc_Command<COptimalBoundingBox>);
    srv->AddInterface(new CLxIfc_Attributes<COptimalBoundingBox>);
    srv->AddInterface(new CLxIfc_AttributesUI<COptimalBoundingBox>);
    lx::AddServer(SRVNAME_COMMAND, srv);
}