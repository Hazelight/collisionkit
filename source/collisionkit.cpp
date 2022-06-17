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
#include <CGAL/convex_hull_3.h>
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

#include <lx_log.hpp>
#include <lxu_log.hpp>

#include <sstream>
#include <string>


namespace PMP = CGAL::Polygon_mesh_processing;

typedef CGAL::Exact_predicates_inexact_constructions_kernel    K;
typedef K::Point_3                                             Point;
typedef CGAL::Surface_mesh<Point>                              Surface_mesh;

using namespace lx_err;

/*
* Reads a CGAL surface mesh and creates a mesh item in the current Modo scene
* 
* Wrote this helper function to not have to repeat mostly same code in all commands.
*/
void create_mesh(const Surface_mesh& input) {
    CLxUser_SceneService scene_service;
    CLxSceneSelection scene_selection;
    CLxUser_Scene scene;
    LXtItemType item_type;
    CLxUser_Item item;
    CLxUser_Mesh mesh;
    CLxUser_Point point_accessor;
    CLxUser_Polygon polygon_accessor;
    CLxUser_ChannelWrite channel_write;
    unsigned channel_index;
    std::vector<LXtPointID> point_ids;
    std::vector<LXtPointID> face_verts;
    LXtVector position;
    LXtPolygonID polygon_id;
    
    scene_service.ItemTypeLookup(LXsTYPE_MESH, &item_type); // Set the item type to mesh 
    scene_selection.Get(scene); // Get the first selected active scene
    scene.ItemAdd(item_type, item); // Add the mesh item to scene
    scene.Channels(LXs_ACTIONLAYER_EDIT, 0.0, channel_write); // Set the channel write object
    item.ChannelLookup(LXsICHAN_MESH_MESH, &channel_index); // Get the channel index for the mesh
    channel_write.ValueObj(item, channel_index, mesh); // Get the mesh object
    point_accessor.fromMesh(mesh); // Get the point accessor
    polygon_accessor.fromMesh(mesh); // Get the polygon accessor
    
    // Iterate over all input vertices and create a new point for the Modo mesh object
    for (Surface_mesh::Vertex_index vertex_index : input.vertices()) {
        Point point = input.point(vertex_index);  // Get the point

        // Set position.xyz from point position
        position[0] = point.x();
        position[1] = point.y();
        position[2] = point.z();

        LXtPointID point_id;
        point_accessor.New(position, &point_id);
        point_ids.push_back(point_id);
    }

    // Iterate over all faces in input mesh,
    for (Surface_mesh::Face_index face_index : input.faces()) {
        // get point ids for face vertices
        for (Surface_mesh::Vertex_index vertex_index : CGAL::vertices_around_face(input.halfedge(face_index), input)) {
            face_verts.push_back(point_ids[vertex_index]);
        }

        // create a point id array which polygon.new requires
        int size = static_cast<int>(face_verts.size()); // Get number of vertices for this face,
        LXtPointID* varr; // Create a pointer to LX Point ID
        varr = new LXtPointID[size]; // Create an array, of same size as number of vertices in face,
        for (int i = 0; i < size; i++)
            varr[i] = face_verts[i];  // Populate the array with Point IDs

        polygon_accessor.New(LXiPTYP_FACE, varr, size, false, &polygon_id); // add a new polygon for the modo mesh item

        face_verts.clear(); // clear vector with face verts
        delete[] varr; // delete the array holding point ids for the face
    }

    mesh.SetMeshEdits(LXf_MESHEDIT_GEOMETRY); // Tell Modo we're finished making edits and to have them applied
}

class COptimalBoundingBox : public CLxBasicCommand {
public:
    COptimalBoundingBox();
    int basic_CmdFlags() LXx_OVERRIDE;
    void basic_Execute(unsigned flags);

private:
    CLxUser_LayerService layer_service;

    CLxUser_Item item;
    CLxUser_Mesh mesh;
    CLxUser_Point point_accessor;
};

// We don't currently need to initialize any inputs so constructor is empty,
COptimalBoundingBox::COptimalBoundingBox() {}

// Notify Modo this command will change scene state and that we want to be able to undo it,
int COptimalBoundingBox::basic_CmdFlags() {
    return LXfCMD_MODEL | LXfCMD_UNDO;
}

void COptimalBoundingBox::basic_Execute(unsigned flags) {
    CLxUser_LayerScan primary_layer;
    check(layer_service.ScanAllocate(LXf_LAYERSCAN_PRIMARY, primary_layer));

    // If we had a mesh selected, keep going otherwise exit early here.
    unsigned int any_primary_layer;
    primary_layer.Count(&any_primary_layer);
    if (!any_primary_layer)
        return;

    // get mesh and item from layer
    check(primary_layer.BaseMeshByIndex(0, mesh));
    check(primary_layer.ItemByIndex(0, item));

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

    CLxUser_Log log; // Create a log object,
    CLxUser_LogService log_service; // Access the log service,
    CLxUser_LogEntry entry; // Create a log entry,
    log_service.GetSubSystem(LXsLOG_LOGSYS, log); // Get the master log system,

    CGAL::Real_timer timer; // Create a timer, and start it to check how long it took to find optimal bounding box
    timer.start();

    std::array<Point, 8> optimal_bounding_box_points;  // The eight points making up the bounding box will be stored to this array,
    CGAL::oriented_bounding_box(input_points, optimal_bounding_box_points,
        CGAL::parameters::use_convex_hull(true)); // Run the CGAL method to get the Optimal Bounding Box for all the points,

    std::stringstream ss; // Create a string stream and push message to log entry
    ss << "Elapsed Time: " << timer.time();
    log_service.NewEntry(LXe_INFO, ss.str().c_str(), entry);
    log.AddEntry(entry);

    Surface_mesh surface_mesh; // Create a cgal mesh from the optimal bound points,
    CGAL::make_hexahedron(optimal_bounding_box_points[0], optimal_bounding_box_points[1], optimal_bounding_box_points[2], optimal_bounding_box_points[3], 
        optimal_bounding_box_points[4], optimal_bounding_box_points[5], optimal_bounding_box_points[6], optimal_bounding_box_points[7], surface_mesh);

    create_mesh(surface_mesh); // Run our helperer function to create a Modo mesh from the CGAL mesh,
}

class CConvexHull : public CLxBasicCommand {
public:
    CConvexHull();
    int basic_CmdFlags() LXx_OVERRIDE;
    void basic_Execute(unsigned flags);

private:
    CLxUser_LayerService layer_service;

    CLxUser_Item item;
    CLxUser_Mesh mesh;
    CLxUser_Point point_accessor;
};

CConvexHull::CConvexHull() {}

int CConvexHull::basic_CmdFlags() {
    return LXfCMD_MODEL | LXfCMD_UNDO;
}

void CConvexHull::basic_Execute(unsigned flags) {
    CLxUser_LayerScan primary_layer;
    layer_service.ScanAllocate(LXf_LAYERSCAN_PRIMARY, primary_layer);

    unsigned int any_primary_layer;
    primary_layer.Count(&any_primary_layer);
    if (!any_primary_layer)
        return;

    // set item and mesh to the primary layer,
    primary_layer.ItemByIndex(0, item);
    primary_layer.BaseMeshByIndex(0, mesh);

    // Store number of points so we can initialize a vector K::Point_3 of same size
    unsigned int num_points;
    mesh.PointCount(&num_points);
    std::vector<Point> input_points(num_points);

    point_accessor.fromMesh(mesh);
    LXtFVector position;
    for (unsigned i = 0; i < num_points; i++) {
        point_accessor.SelectByIndex(i);
        point_accessor.Pos(position);
        input_points[i] = Point(position[0], position[1], position[2]);
    }

    primary_layer.Apply();
    primary_layer.clear();
    primary_layer = NULL;

    // generate the convex hull mesh,
    Surface_mesh surface_mesh;
    CGAL::convex_hull_3(input_points.begin(), input_points.end(), surface_mesh);

    create_mesh(surface_mesh);
}

void initialize() {
    CLxGenericPolymorph* ubx;
    CLxGenericPolymorph* ucx;

    ubx = new CLxPolymorph<COptimalBoundingBox>;
    ubx->AddInterface(new CLxIfc_Command<COptimalBoundingBox>);
    ubx->AddInterface(new CLxIfc_Attributes<COptimalBoundingBox>);
    ubx->AddInterface(new CLxIfc_AttributesUI<COptimalBoundingBox>);
    lx::AddServer("ubx.new", ubx);

    ucx = new CLxPolymorph<CConvexHull>;
    ucx->AddInterface(new CLxIfc_Command<CConvexHull>);
    ucx->AddInterface(new CLxIfc_Attributes<CConvexHull>);
    ucx->AddInterface(new CLxIfc_AttributesUI<CConvexHull>);
    lx::AddServer("ucx.new", ucx);

}