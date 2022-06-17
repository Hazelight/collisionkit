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

#include <lxidef.h> // LXsICHAN_MESH_MESH, symbol used for getting mesh channel
#include <lx_action.hpp> // Channel Write,
#include <lx_layer.hpp> // Layer Scan & Service,
#include <lx_mesh.hpp> // Mesh, Point & Polygon
#include <lxu_select.hpp> // Scene Selection
#include <lxu_command.hpp> // Command

typedef CGAL::Exact_predicates_inexact_constructions_kernel    K;
typedef K::Point_3                                             Point;
typedef CGAL::Surface_mesh<Point>                              Surface_mesh;

using namespace lx_err;

/*
* Populate the vector points with data from the first selected layer in Modo,
*/
void get_selected_points(std::vector<Point>* points) {
    CLxUser_LayerService layer_service;
    CLxUser_LayerScan layer_scan;
    unsigned layer_count;
    unsigned point_count;
    CLxUser_Mesh mesh;
    CLxUser_Point point;
    LXtFVector position;
    
    layer_service.ScanAllocate(LXf_LAYERSCAN_PRIMARY, layer_scan);
    layer_scan.Count(&layer_count);

    if (!layer_count)
        return;

    layer_scan.BaseMeshByIndex(0, mesh);
    point.fromMesh(mesh);
    mesh.PointCount(&point_count);
    points->clear();
    for (unsigned index = 0; index < point_count; index++) {
        point.SelectByIndex(index);
        point.Pos(position);
        points->push_back(Point(position[0], position[1], position[2]));
    }

    layer_scan.Apply();
    layer_scan.clear();
}

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
};

COptimalBoundingBox::COptimalBoundingBox() {}

int COptimalBoundingBox::basic_CmdFlags() {
    return LXfCMD_MODEL | LXfCMD_UNDO; // Notify Modo this command will change scene state and that we want to be able to undo it,
}

void COptimalBoundingBox::basic_Execute(unsigned flags) {
    std::vector<Point> input_points;
    get_selected_points(&input_points);

    std::array<Point, 8> optimal_bounding_box_points;  // The eight points making up the bounding box will be stored to this array,
    CGAL::oriented_bounding_box(input_points, optimal_bounding_box_points,
        CGAL::parameters::use_convex_hull(true)); // Run the CGAL method to get the Optimal Bounding Box for all the points,

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
};

CConvexHull::CConvexHull() {}

int CConvexHull::basic_CmdFlags() {
    return LXfCMD_MODEL | LXfCMD_UNDO;
}

void CConvexHull::basic_Execute(unsigned flags) {
    std::vector<Point> input_points;
    get_selected_points(&input_points);

    Surface_mesh convex_hull; // From points generate a CGAL mesh,
    CGAL::convex_hull_3(input_points.begin(), input_points.end(), convex_hull);

    create_mesh(convex_hull); // Create a Modo mesh of the Convex Hull
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