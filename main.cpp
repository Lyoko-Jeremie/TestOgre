#include <iostream>
#include <array>
#include <memory>
#include <utility>
#include <vector>
#include <memory>
#include <map>
#include <Ogre.h>
#include <OgreApplicationContext.h>
#include <OgreInput.h>
#include <OgreRTShaderSystem.h>
#include <OgreFileSystemLayer.h>
#include <OgreFileSystem.h>
#include <OgreMeshManager.h>
#include <OgreCameraMan.h>
#include <OgreCamera.h>
#include <OgreImGuiOverlay.h>
#include <OgreImGuiInputListener.h>
#include <OgreOverlayManager.h>
#include <OgreOverlaySystem.h>

//#include "OgreUnifiedHighLevelGpuProgram.h"
#include "Bullet/OgreBullet.h"
#include "Bullet/BodyHelper.h"


#include <boost/log/trivial.hpp>
#include <boost/assert.hpp>

namespace boost {
    void assertion_failed(char const *expr, char const *function, char const *file, long line) {
        BOOST_LOG_TRIVIAL(error)
            << "assertion_failed : [" << expr << "]"
            << " on function [" << function << "]"
            << " on file [" << file << "]"
            << " at line [" << line << "]";
        std::abort();
    }

    void assertion_failed_msg(char const *expr, char const *msg, char const *function, char const *file, long line) {
        BOOST_LOG_TRIVIAL(error)
            << "assertion_failed_msg : [" << expr << "]"
            << " msg [" << msg << "]"
            << " on function [" << function << "]"
            << " on file [" << file << "]"
            << " at line [" << line << "]";
        std::abort();
    }
}


// https://github.com/ilmola/generator
#include <generator/generator.hpp>

#include "ttf2mesh/ttf2mesh.h"

#include "./Bullet/BulletMemoryContainer.h"


class KeyHandler : public OgreBites::InputListener {
    bool keyPressed(const OgreBites::KeyboardEvent &evt) override {
        if (evt.keysym.sym == OgreBites::SDLK_ESCAPE) {
            Ogre::Root::getSingleton().queueEndRendering();
        }
        return true;
    }
};


class ImGuiDrawHandler : public Ogre::FrameListener {

public:

    ImGuiIO &io;

    // must create after create `Ogre::ImGuiOverlay`
    ImGuiDrawHandler() : io(ImGui::GetIO()) {
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls


        // Setup Dear ImGui style
        ImGui::StyleColorsDark();
//        ImGui::StyleColorsLight();

        if (false) {
            ImFontConfig config;
//            config.OversampleH = 2;
//            config.OversampleV = 1;
            config.GlyphExtraSpacing.x = 1.0f;
            std::vector<ImFont *> fontsList{
//            io.Fonts->AddFontDefault(),
                    io.Fonts->AddFontFromFileTTF(R"(D:\IDMDownloads\SourceHanSansCN\SourceHanSansCN-Normal.otf)", 18.0f,
                                                 &config, io.Fonts->GetGlyphRangesChineseFull()),
//                    io.Fonts->AddFontFromFileTTF(ppp->config().font_path.c_str(), 21.0f,
//                                                 &config, io.Fonts->GetGlyphRangesChineseFull()),
            };
            io.Fonts->Build();
            for (const auto &a: fontsList) {
//                if (!a) {
//                    // cannot read fonts
//                    clear();
//                    OwlImGuiService::safe_exit();
//                    return -2;
//                }
                std::cout << a->GetDebugName() << " " << a << " " << a->IsLoaded() << std::endl;
                a->ContainerAtlas->Build();
            }
            io.FontDefault = fontsList.at(0);
        }
    }

    // Our state
    bool show_demo_window = false;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    bool frameStarted(const Ogre::FrameEvent &evt) override {

        Ogre::ImGuiOverlay::NewFrame();

        // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
        if (show_demo_window)
            ImGui::ShowDemoWindow(&show_demo_window);

        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
        {
            static float f = 0.0f;
            static int counter = 0;

            ImGui::Begin(
                    "Hello, world!");                          // Create a window called "Hello, world!" and append into it.

            ImGui::Text(
                    "This is some useful text.");               // Display some text (you can use a format strings too)
            ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
            ImGui::Checkbox("Another Window", &show_another_window);

            ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::ColorEdit3("clear color", (float *) &clear_color); // Edit 3 floats representing a color

            if (ImGui::Button(
                    "Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
                counter++;
            ImGui::SameLine();
            ImGui::Text("counter = %d", counter);

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            ImGui::End();
        }

        // 3. Show another simple window.
        if (show_another_window) {
            ImGui::Begin("Another Window",
                         &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
            ImGui::Text("Hello from another window!");
            if (ImGui::Button("Close Me"))
                show_another_window = false;
            ImGui::End();
        }

        return true;
    }

};

//class BulletManager : public boost::enable_shared_from_this<BulletManager> {
//public:
//
//    // TODO use memory pool
//    std::unique_ptr<Ogre::Bullet::DynamicsWorld> mDynWorld;
//    std::unique_ptr<Ogre::Bullet::DebugDrawer> mDbgDraw;
//    std::unique_ptr<btStaticPlaneShape> btStaticPlaneShape_;
//    std::unique_ptr<btMotionState> btStaticPlaneMotionState_;
//    std::unique_ptr<btRigidBody> btStaticPlaneRigidBody_;
//
//    std::unique_ptr<CollisionStateContainer> collisionStateContainer{std::make_unique<CollisionStateContainer>()};
//    std::unique_ptr<CollisionShapeContainer> collisionShapeContainer{std::make_unique<CollisionShapeContainer>()};
//    std::unique_ptr<RigidObjectContainer> rigidObjectContainer{std::make_unique<RigidObjectContainer>()};
//
//    BulletManager(Ogre::SceneManager *scnMgr)
//            : mDynWorld(std::make_unique<Ogre::Bullet::DynamicsWorld>(Ogre::Vector3(0, -9.8, 0))),
//              mDbgDraw(std::make_unique<Ogre::Bullet::DebugDrawer>(
//                      scnMgr->getRootSceneNode(),
//                      mDynWorld->getBtWorld())),
//              btStaticPlaneShape_(std::make_unique<btStaticPlaneShape>(btVector3(0, 1, 0), 0)),
//              btStaticPlaneMotionState_(std::make_unique<btDefaultMotionState>()),
//              btStaticPlaneRigidBody_(std::make_unique<btRigidBody>(
//                      btRigidBody::btRigidBodyConstructionInfo{
//                              0, btStaticPlaneMotionState_.get(), btStaticPlaneShape_.get()
//                      }
//              )) {
//
//        //        mDynWorld->addRigidBody(5, player, Ogre::Bullet::CT_SPHERE);
//        //        mDynWorld->addRigidBody(0, level, Ogre::Bullet::CT_TRIMESH);
//
//        mDynWorld->getBtWorld()->addRigidBody(btStaticPlaneRigidBody_.get());
//    }
//
//};

//class BulletHandler : public Ogre::FrameListener {
//public:
//
//    boost::shared_ptr<BulletManager> bulletManager;
//
//    explicit BulletHandler(Ogre::SceneManager *scnMgr) : bulletManager(boost::make_shared<BulletManager>(scnMgr)) {}
//
//    bool frameStarted(const Ogre::FrameEvent &evt) override {
//
//        bulletManager->mDynWorld->getBtWorld()->stepSimulation(evt.timeSinceLastFrame, 10);
//        bulletManager->mDbgDraw->update();
//
//        return true;
//    }
//
//};


// https://wiki.ogre3d.org/Generating+A+Mesh
void createColourCube() {
    /// Create the mesh via the MeshManager
    Ogre::MeshPtr msh = Ogre::MeshManager::getSingleton().createManual("ColourCube", "General");

    /// Create one submesh
    Ogre::SubMesh *sub = msh->createSubMesh();

    const float sqrt13 = 0.577350269f; /* sqrt(1/3) */

    /// Define the vertices (8 vertices, each have 3 floats for position and 3 for normal)
    const size_t nVertices = 8;
    const size_t vbufCount = 3 * 2 * nVertices;
    float vertices[vbufCount] = {
            -100.0, 100.0, -100.0,        //0 position
            -sqrt13, sqrt13, -sqrt13,     //0 normal
            100.0, 100.0, -100.0,         //1 position
            sqrt13, sqrt13, -sqrt13,      //1 normal
            100.0, -100.0, -100.0,        //2 position
            sqrt13, -sqrt13, -sqrt13,     //2 normal
            -100.0, -100.0, -100.0,       //3 position
            -sqrt13, -sqrt13, -sqrt13,    //3 normal
            -100.0, 100.0, 100.0,         //4 position
            -sqrt13, sqrt13, sqrt13,      //4 normal
            100.0, 100.0, 100.0,          //5 position
            sqrt13, sqrt13, sqrt13,       //5 normal
            100.0, -100.0, 100.0,         //6 position
            sqrt13, -sqrt13, sqrt13,      //6 normal
            -100.0, -100.0, 100.0,        //7 position
            -sqrt13, -sqrt13, sqrt13,     //7 normal
    };

//    Ogre::RenderSystem *rs = Ogre::Root::getSingleton().getRenderSystem();
    Ogre::RGBA colours[nVertices];
    Ogre::RGBA *pColour = colours;
    // Use render system to convert colour value since colour packing varies
    *(pColour++) = Ogre::ColourValue(1.0, 0.0, 0.0).getAsBYTE(); //0 colour
    *(pColour++) = Ogre::ColourValue(1.0, 1.0, 0.0).getAsBYTE(); //1 colour
    *(pColour++) = Ogre::ColourValue(0.0, 1.0, 0.0).getAsBYTE(); //2 colour
    *(pColour++) = Ogre::ColourValue(0.0, 0.0, 0.0).getAsBYTE(); //3 colour
    *(pColour++) = Ogre::ColourValue(1.0, 0.0, 1.0).getAsBYTE(); //4 colour
    *(pColour++) = Ogre::ColourValue(1.0, 1.0, 1.0).getAsBYTE(); //5 colour
    *(pColour++) = Ogre::ColourValue(0.0, 1.0, 1.0).getAsBYTE(); //6 colour
    *(pColour++) = Ogre::ColourValue(0.0, 0.0, 1.0).getAsBYTE(); //7 colour
//    rs->convertColourValue(Ogre::ColourValue(1.0, 0.0, 0.0), pColour++); //0 colour
//    rs->convertColourValue(Ogre::ColourValue(1.0, 1.0, 0.0), pColour++); //1 colour
//    rs->convertColourValue(Ogre::ColourValue(0.0, 1.0, 0.0), pColour++); //2 colour
//    rs->convertColourValue(Ogre::ColourValue(0.0, 0.0, 0.0), pColour++); //3 colour
//    rs->convertColourValue(Ogre::ColourValue(1.0, 0.0, 1.0), pColour++); //4 colour
//    rs->convertColourValue(Ogre::ColourValue(1.0, 1.0, 1.0), pColour++); //5 colour
//    rs->convertColourValue(Ogre::ColourValue(0.0, 1.0, 1.0), pColour++); //6 colour
//    rs->convertColourValue(Ogre::ColourValue(0.0, 0.0, 1.0), pColour++); //7 colour

    /// Define 12 triangles (two triangles per cube face)
    /// The values in this table refer to vertices in the above table
    const size_t ibufCount = 36;
    unsigned short faces[ibufCount] = {
            0, 2, 3,
            0, 1, 2,
            1, 6, 2,
            1, 5, 6,
            4, 6, 5,
            4, 7, 6,
            0, 7, 4,
            0, 3, 7,
            0, 5, 1,
            0, 4, 5,
            2, 7, 3,
            2, 6, 7
    };

    /// Create vertex data structure for 8 vertices shared between submeshes
    msh->sharedVertexData = new Ogre::VertexData();
    msh->sharedVertexData->vertexCount = nVertices;

    /// Create declaration (memory format) of vertex data
    Ogre::VertexDeclaration *decl = msh->sharedVertexData->vertexDeclaration;
    size_t offset = 0;
    // 1st buffer
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    /// Allocate vertex buffer of the requested number of vertices (vertexCount)
    /// and bytes per vertex (offset)
    Ogre::HardwareVertexBufferSharedPtr vbuf =
            Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
                    offset, msh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    /// Upload the vertex data to the card
    vbuf->writeData(0, vbuf->getSizeInBytes(), vertices, true);

    /// Set vertex buffer binding so buffer 0 is bound to our vertex buffer
    Ogre::VertexBufferBinding *bind = msh->sharedVertexData->vertexBufferBinding;
    bind->setBinding(0, vbuf);

    // 2nd buffer
    offset = 0;
    decl->addElement(1, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_COLOUR);
    /// Allocate vertex buffer of the requested number of vertices (vertexCount)
    /// and bytes per vertex (offset)
    vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
            offset, msh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    /// Upload the vertex data to the card
    vbuf->writeData(0, vbuf->getSizeInBytes(), colours, true);

    /// Set vertex buffer binding so buffer 1 is bound to our colour buffer
    bind->setBinding(1, vbuf);

    /// Allocate index buffer of the requested number of vertices (ibufCount)
    Ogre::HardwareIndexBufferSharedPtr ibuf = Ogre::HardwareBufferManager::getSingleton().
            createIndexBuffer(
            Ogre::HardwareIndexBuffer::IT_16BIT,
            ibufCount,
            Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

    /// Upload the index data to the card
    ibuf->writeData(0, ibuf->getSizeInBytes(), faces, true);

    /// Set parameters of the submesh
    sub->useSharedVertices = true;
    sub->indexData->indexBuffer = ibuf;
    sub->indexData->indexCount = ibufCount;
    sub->indexData->indexStart = 0;

    /// Set bounding information (for culling)
    msh->_setBounds(Ogre::AxisAlignedBox(-100, -100, -100, 100, 100, 100));
    msh->_setBoundingSphereRadius(Ogre::Math::Sqrt(3 * 100 * 100));

    /// Notify -Mesh object that it has been loaded
    msh->load();
}


void createBoxMesh() {
    /// Create the mesh via the MeshManager
    Ogre::MeshPtr msh = Ogre::MeshManager::getSingleton().createManual("ColourBoxMesh", "General");


    gml::dvec3 sizeBox{100., 100., 100.};
    gml::ivec3 segmentBox{1, 1, 1};
//    auto boxMesh = generator::BoxMesh{sizeBox, segmentBox};
    auto boxMesh = generator::SphereMesh{100.0};


    auto vert = boxMesh.vertices();
    std::vector<float> vertices;
    const size_t nVertices = generator::count(vert);
    vertices.reserve(nVertices * 3 * 2);
    std::cout << "vertices:" << nVertices << std::endl;
    for (const auto &vertex: vert) {
//        vertex.normal;
//        vertex.position;
//        vertex.texCoord;
        auto &p = vertex.position;
        std::cout << "\tp:" << p << std::endl;
        vertices.emplace_back((float) p[0]);
        vertices.emplace_back((float) p[1]);
        vertices.emplace_back((float) p[2]);
        auto &n = vertex.normal;
        std::cout << "\tn:" << n << std::endl;
        vertices.emplace_back((float) n[0]);
        vertices.emplace_back((float) n[1]);
        vertices.emplace_back((float) n[2]);
    };
    std::cout << std::endl;


    std::vector<Ogre::RGBA> colours;
    colours.assign(nVertices, Ogre::ColourValue(1.0, 1.0, 1.0, 1.0).getAsBYTE());

    auto tr = boxMesh.triangles();
    std::vector<unsigned short> faces;
    const size_t ibufCount = generator::count(tr);
    faces.reserve(ibufCount);
    std::cout << "faces:" << ibufCount << std::endl;
    for (const auto &vertex: tr) {
        auto &v = vertex.vertices;
        std::cout << "\tv:" << v << std::endl;
        faces.emplace_back((unsigned short) v[0]);
        faces.emplace_back((unsigned short) v[1]);
        faces.emplace_back((unsigned short) v[2]);
    };
    std::cout << std::endl;

    /// Create one submesh
    Ogre::SubMesh *sub = msh->createSubMesh();

    /// Create vertex data structure for 8 vertices shared between submeshes
    msh->sharedVertexData = new Ogre::VertexData();
    msh->sharedVertexData->vertexCount = nVertices;

    /// Create declaration (memory format) of vertex data
    Ogre::VertexDeclaration *decl = msh->sharedVertexData->vertexDeclaration;
    size_t offset = 0;
    // 1st buffer
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    /// Allocate vertex buffer of the requested number of vertices (vertexCount)
    /// and bytes per vertex (offset)
    Ogre::HardwareVertexBufferSharedPtr vbuf =
            Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
                    offset, msh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    /// Upload the vertex data to the card
    vbuf->writeData(0, vbuf->getSizeInBytes(), vertices.data(), true);

    /// Set vertex buffer binding so buffer 0 is bound to our vertex buffer
    Ogre::VertexBufferBinding *bind = msh->sharedVertexData->vertexBufferBinding;
    bind->setBinding(0, vbuf);

    // 2nd buffer
    offset = 0;
    decl->addElement(1, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_COLOUR);
    /// Allocate vertex buffer of the requested number of vertices (vertexCount)
    /// and bytes per vertex (offset)
    vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
            offset, msh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    /// Upload the vertex data to the card
    vbuf->writeData(0, vbuf->getSizeInBytes(), colours.data(), true);

    /// Set vertex buffer binding so buffer 1 is bound to our colour buffer
    bind->setBinding(1, vbuf);

    /// Allocate index buffer of the requested number of vertices (ibufCount)
    Ogre::HardwareIndexBufferSharedPtr ibuf = Ogre::HardwareBufferManager::getSingleton().
            createIndexBuffer(
            Ogre::HardwareIndexBuffer::IT_16BIT,
            ibufCount,
            Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

    /// Upload the index data to the card
    ibuf->writeData(0, ibuf->getSizeInBytes(), faces.data(), true);

    /// Set parameters of the submesh
    sub->useSharedVertices = true;
    sub->indexData->indexBuffer = ibuf;
    sub->indexData->indexCount = ibufCount;
    sub->indexData->indexStart = 0;

    /// Set bounding information (for culling)
//    msh->_setBounds(Ogre::AxisAlignedBox(-100, -100, -100, 100, 100, 100));
//    msh->_setBoundingSphereRadius(Ogre::Math::Sqrt(3 * 100 * 100));
    {
        auto mini = -sizeBox;
        auto max = sizeBox;
        Ogre::AxisAlignedBox aab{
                (float) mini[0], (float) mini[1], (float) mini[2],
                (float) max[0], (float) max[1], (float) max[2]
        };
        std::cout << "aab:" << aab << std::endl;
        msh->_setBounds(aab);
        std::cout << "getBounds:" << msh->getBounds() << std::endl;
    }
    {
        auto q = sizeBox * sizeBox;
        std::cout << "q:" << q << std::endl;
        auto r = Ogre::Math::Sqrt((float) (q[0] + q[1] + q[2]));
        std::cout << "r:" << r << std::endl;
        msh->_setBoundingSphereRadius(r);
    }

    /// Notify -Mesh object that it has been loaded
    msh->load();
}

Ogre::MeshPtr createMeshObject(
        Ogre::SceneManager *scnMgr,
        const generator::AnyMesh &mesh,
        const std::string &meshName
) {
    // https://wiki.ogre3d.org/MadMarx+Tutorial+4

//    gml::dvec3 sizeBox{100., 100., 100.};
//    gml::ivec3 segmentBox{1, 1, 1};
////    auto boxMesh = generator::BoxMesh{sizeBox, segmentBox};
////    auto boxMesh = generator::SphereMesh{100.0};
//    auto boxMesh = generator::SphereMesh{
//            100.0,
//            6,
//            6
//    };

    {
//        if (scnMgr->hasManualObject(meshName)) {
//            auto o = scnMgr->getManualObject(meshName);
//            if (o) {
//                std::cout << "o exist : " << meshName << std::endl;
//            }
//            auto m = Ogre::MeshManager::getSingleton().getByName(meshName);
//            if (m) {
//                std::cout << "m exist : " << meshName << std::endl;
//                return m;
//            }
//        }
        auto m = Ogre::MeshManager::getSingleton().getByName(meshName);
        if (m) {
            std::cout << "m exist : " << meshName << std::endl;
            return m;
        }
    }

    auto manualObject = scnMgr->createManualObject();
    manualObject->setDynamic(false);


    auto vert = mesh.vertices();
    const size_t nVertices = generator::count(vert);
    std::cout << "vertices:" << nVertices << std::endl;


    manualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    for (const auto &vertex: vert) {
//        vertex.normal;
//        vertex.position;
//        vertex.texCoord;
        auto &p = vertex.position;
        std::cout << "\tp:" << p << std::endl;
        auto &n = vertex.normal;
        std::cout << "\tn:" << n << std::endl;
        manualObject->position(p[0], p[1], p[2]);// a vertex
        manualObject->normal(n[0], n[1], n[2]);
        manualObject->colour(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));
    };
    std::cout << std::endl;


    auto tr = mesh.triangles();
    const size_t ibufCount = generator::count(tr);
    std::cout << "faces:" << ibufCount << std::endl;
    for (const auto &vertex: tr) {
        auto &v = vertex.vertices;
        std::cout << "\tv:" << v << std::endl;
        manualObject->triangle(v[0], v[1], v[2]);
    };
    std::cout << std::endl;

    manualObject->end();

    auto msh = manualObject->convertToMesh(meshName);
    std::cout << "getBounds:" << msh->getBounds() << std::endl;
    std::cout << "getBoneBoundingRadius:" << msh->getBoneBoundingRadius() << std::endl;
    std::cout << "getBoundingSphereRadius:" << msh->getBoundingSphereRadius() << std::endl;

    return msh;
};

struct TtfMeshFactory : public std::enable_shared_from_this<TtfMeshFactory> {
    std::shared_ptr<ttf_t> font{nullptr, ttf_free};

    std::string getFontName() const {
        auto name = std::string{font->names.full_name};
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    }

    struct TtfMeshSize {
        double x;
        double y;
        double z;

        [[nodiscard]] Ogre::Vector3 toOgreVec3() const {
            return Ogre::Vector3{
                    static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)
            };
        }
    };

    struct Info2d : public std::enable_shared_from_this<Info2d> {
        ttf_glyph_t *glyph{nullptr};
        std::shared_ptr<ttf_t> font{nullptr, ttf_free};
        std::shared_ptr<ttf_mesh_t> mesh{nullptr, ttf_free_mesh};

        Info2d(ttf_glyph_t *_glyph, std::shared_ptr<ttf_t> _font, ttf_mesh_t *_mesh)
                : glyph(_glyph), font(std::move(_font)),
                  mesh(std::make_shared<ttf_mesh_t>()) {
            mesh.reset(_mesh, ttf_free_mesh);
        }

        Info2d(ttf_glyph_t *_glyph, std::shared_ptr<ttf_t> _font, std::shared_ptr<ttf_mesh_t> _mesh)
                : glyph(_glyph), font(std::move(_font)), mesh(std::move(_mesh)) {}

        TtfMeshSize getPos() const {
            return {
                    -(glyph->xbounds[0] + glyph->xbounds[1]) / 2, -glyph->ybounds[0], 0.0f
            };
        }

        TtfMeshSize getSize() const {
            return {
                    -(glyph->xbounds[0] + glyph->xbounds[1]) / 2, -glyph->ybounds[0], 0.0f
            };
        }
    };

    struct Info3d : public std::enable_shared_from_this<Info3d> {
        ttf_glyph_t *glyph{nullptr};
        std::shared_ptr<ttf_t> font{nullptr, ttf_free};
        std::shared_ptr<ttf_mesh3d_t> mesh3d{nullptr, ttf_free_mesh3d};
        float depth;

        Info3d(ttf_glyph_t *_glyph, std::shared_ptr<ttf_t> _font, ttf_mesh3d_t *_mesh3d, float _depth)
                : glyph(_glyph), font(std::move(_font)),
                  mesh3d(std::make_shared<ttf_mesh3d_t>()),
                  depth(_depth) {
            mesh3d.reset(_mesh3d, ttf_free_mesh3d);
        }

        Info3d(ttf_glyph_t *_glyph, std::shared_ptr<ttf_t> _font, std::shared_ptr<ttf_mesh3d_t> _mesh3d, float _depth)
                : glyph(_glyph), font(std::move(_font)), mesh3d(std::move(_mesh3d)), depth(_depth) {}

        TtfMeshSize getPos() const {
            return {
                    -(glyph->xbounds[0] + glyph->xbounds[1]) / 2, -glyph->ybounds[0], 0.0f
            };
        }

        TtfMeshSize getSize() const {
            return {
                    glyph->xbounds[0] + glyph->xbounds[1],
                    glyph->ybounds[0] + glyph->ybounds[1],
                    depth
            };
        }
    };


    std::map<int, std::weak_ptr<Info3d>> meshInfo3dPool;

    bool load_system_font(std::vector<std::string> fontDir, std::string mask = "*") {
        // list all system fonts by filename mask:

//        ttf_t **list = ttf_list_system_fonts("DejaVuSans*|Ubuntu*|FreeSerif*|Arial*|Cour*");
//        std::vector<std::string> fontDir{
//                R"(d:\IDMDownloads\source-han-sans-ttf-2.002.1\)"
//        };
        std::unique_ptr<const char *[]> fontDirPtr{new const char *[fontDir.size()]};
        for (size_t i = 0; i != fontDir.size(); ++i) {
            fontDirPtr[i] = fontDir.at(i).data();
        }
        ttf_t **list = ttf_list_fonts(fontDirPtr.get(), fontDir.size(), mask.c_str());
        if (list == nullptr) return false; // no memory in system
        if (list[0] == nullptr) return false; // no fonts were found

        // load the first font from the list

        ttf_t *ptr = nullptr;
        ttf_load_from_file(list[0]->filename, &ptr, false);
        ttf_free_list(list);
        if (ptr == nullptr) return false;
        font.reset(ptr);

        printf("font \"%s\" loaded\n", font->names.full_name);
        return true;
    }

    std::shared_ptr<Info2d> choose_glyph(wchar_t symbol) {
        // find a glyph in the font file

        int index = ttf_find_glyph(font.get(), symbol);
        if (index < 0) return nullptr;

        // make mesh object from the glyph

        ttf_mesh_t *out;
        if (ttf_glyph2mesh(&font->glyphs[index], &out, TTF_QUALITY_NORMAL, TTF_FEATURES_DFLT) != TTF_DONE)
            return nullptr;

        // if successful, release the previous object and save the state

        std::shared_ptr<Info2d> data = std::make_shared<Info2d>(
                &font->glyphs[index], font, out
        );
        return data;
    }

    std::shared_ptr<Info3d> choose_glyph_3d(wchar_t symbol, float depth = 0.1f) {
        if (depth <= 0) {
            depth = 0.1f;
        }
        // find a glyph in the font file

        int index = ttf_find_glyph(font.get(), symbol);
        if (index < 0) return nullptr;
        if (meshInfo3dPool.contains(index)) {
            auto wp = meshInfo3dPool.at(index);
            auto p = wp.lock();
            if (p) {
                return p;
            }
            // this is invalid
            meshInfo3dPool.erase(index);
        }

        // make 3d object from the glyph

        ttf_mesh3d_t *out;
        if (ttf_glyph2mesh3d(&font->glyphs[index], &out, TTF_QUALITY_NORMAL, TTF_FEATURES_DFLT, depth) != TTF_DONE)
            return nullptr;

        // if successful, release the previous object and save the state

        std::shared_ptr<Info3d> data = std::make_shared<Info3d>(
                &font->glyphs[index], font, out, depth
        );
        return data;
    }

};

struct TtfMeshData : public std::enable_shared_from_this<TtfMeshData> {
    std::shared_ptr<TtfMeshFactory::Info3d> info3D;
    Ogre::MeshPtr meshPtr;

    TtfMeshData(std::shared_ptr<TtfMeshFactory::Info3d> info3D_, const Ogre::MeshPtr &meshPtr_)
            : info3D(std::move(info3D_)), meshPtr(meshPtr_) {}
};

std::shared_ptr<TtfMeshData> createTtfMesh(Ogre::SceneManager *scnMgr,
                                           const std::shared_ptr<TtfMeshFactory> &ttfMeshFactory,
                                           wchar_t symbol = 'A',
                                           float depth = 0.1f) {
    // https://github.com/fetisov/ttf2mesh/blob/master/examples/src/simple.c

    auto G = ttfMeshFactory->choose_glyph_3d(symbol, depth);
//    glTranslatef(-(glyph->xbounds[0] + glyph->xbounds[1]) / 2, -glyph->ybounds[0], 0.0f);

    if (!G) {
        std::cout << "(!G)" << std::endl;
        return nullptr;
    }

    auto sizeG = G->getSize();
    G->mesh3d->nvert;
    G->mesh3d->vert;
    G->mesh3d->normals;
    G->mesh3d->nfaces;
    G->mesh3d->faces;


    std::vector<std::string> nameList{
            "ManualObjectName",
            "MeshName",
    };
    {
        auto fName = ttfMeshFactory->getFontName();
        nameList.at(0) += '_';
        nameList.at(0) += fName;
        nameList.at(0) += symbol;
        nameList.at(1) += '_';
        nameList.at(1) += fName;
        nameList.at(1) += symbol;
    }

    {
        if (scnMgr->hasManualObject(nameList.at(0))) {
            auto o = scnMgr->getManualObject(nameList.at(0));
            if (o) {
                std::cout << "o exist : " << nameList.at(0) << std::endl;
            }
            auto m = Ogre::MeshManager::getSingleton().getByName(nameList.at(1));
            if (m) {
                std::cout << "m exist : " << nameList.at(1) << std::endl;
                return std::make_shared<TtfMeshData>(
                        G,
                        m
                );
            }
        }
    }

    auto manualObject = scnMgr->createManualObject(nameList.at(0));
    manualObject->setDynamic(false);


    const size_t nVertices = G->mesh3d->nvert;
    std::cout << "vertices:" << nVertices << std::endl;

    manualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    for (size_t i = 0; i != nVertices; ++i) {
//        vertex.normal;
//        vertex.position;
//        vertex.texCoord;

//        auto &p = vertex.position;
//        std::cout << "\tp:" << p << std::endl;
//        auto &n = vertex.normal;
//        std::cout << "\tn:" << n << std::endl;
//        manualObject->position(p[0], p[1], p[2]);// a vertex
//        manualObject->normal(n[0], n[1], n[2]);
//        manualObject->colour(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));


        manualObject->position(
                (G->mesh3d->vert[i]).x,
                (G->mesh3d->vert[i]).y,
                (G->mesh3d->vert[i]).z
        );
        manualObject->normal(
                (G->mesh3d->normals[i]).x,
                (G->mesh3d->normals[i]).y,
                (G->mesh3d->normals[i]).z
        );
        manualObject->colour(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));
    };
    std::cout << std::endl;


    const size_t ibufCount = G->mesh3d->nfaces;
    std::cout << "faces:" << ibufCount << std::endl;
    for (size_t i = 0; i != ibufCount; ++i) {
//        auto &v = vertex.vertices;
//        std::cout << "\tv:" << v << std::endl;
//        manualObject->triangle(v[0], v[1], v[2]);
        manualObject->triangle(
                (G->mesh3d->faces[i]).v1,
                (G->mesh3d->faces[i]).v2,
                (G->mesh3d->faces[i]).v3
        );
    };
    std::cout << std::endl;


    manualObject->end();

    Ogre::MeshPtr msh = manualObject->convertToMesh(nameList.at(1));
    std::cout << "getBounds:" << msh->getBounds() << std::endl;
    std::cout << "getBoneBoundingRadius:" << msh->getBoneBoundingRadius() << std::endl;
    std::cout << "getBoundingSphereRadius:" << msh->getBoundingSphereRadius() << std::endl;

    return std::make_shared<TtfMeshData>(
            G,
            msh
    );
}


BulletMemoryContainer::BulletMemoryContainerManager &
aa(boost::shared_ptr<BulletMemoryContainer::BulletMemoryContainerManager> P) {
    return *P;
}

auto bb(boost::shared_ptr<BulletMemoryContainer::BulletMemoryContainerManager> P, size_t id) {
    return P->getCollisionShape(id);
}

int main() {
    std::cout << "Hello, World!" << std::endl;


    BulletMemoryPool::setup();

    auto bulletMemoryContainerManager = BulletMemoryContainer::BulletMemoryContainerManager::create(
            BulletMemoryPool::gpMemoryPoolManager
    );

    auto dynamicsWorld = Ogre::Bullet::DynamicsWorld::create(
            bulletMemoryContainerManager, {0, -10, 0}
    );

    Ogre::Bullet::BodyHelper::createInfiniteGround(
            dynamicsWorld, "InfiniteGround-1"
    );


    auto ttfMeshFactory = std::make_shared<TtfMeshFactory>();
    std::vector<std::string> fontDir{
            R"(d:\IDMDownloads\source-han-sans-ttf-2.002.1\test\)"
    };
    if (!ttfMeshFactory->load_system_font(fontDir, "*")) {
        std::cout << "(!ttfMeshFactory->load_system_font())" << std::endl;
        return -1;
    }

    std::string title{"OgreTutorialApp"};

    boost::shared_ptr<OgreBites::ApplicationContext> ctx = boost::make_shared<OgreBites::ApplicationContext>(title);
    ctx->getFSLayer().setHomePath({"."});
    ctx->getFSLayer().setConfigPaths({"ogre.cfg"});
    ctx->initApp();


    // get a pointer to the already created root
    Ogre::Root *root = ctx->getRoot();
    Ogre::SceneManager *scnMgr = root->createSceneManager();

    // register our scene with the RTSS
    Ogre::RTShader::ShaderGenerator *shadergen = Ogre::RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(scnMgr);


    dynamicsWorld->initDebugDrawer(scnMgr->getRootSceneNode());
    dynamicsWorld->setDebugMode(
            btIDebugDraw::DebugDrawModes::DBG_DrawWireframe |
            btIDebugDraw::DebugDrawModes::DBG_DrawAabb |
            btIDebugDraw::DebugDrawModes::DBG_DrawConstraintLimits |
            btIDebugDraw::DebugDrawModes::DBG_DrawContactPoints |
            btIDebugDraw::DebugDrawModes::DBG_DrawFrames |
            btIDebugDraw::DebugDrawModes::DBG_DrawNormals |
            btIDebugDraw::DebugDrawModes::DBG_DrawText |
            btIDebugDraw::DebugDrawModes::DBG_EnableCCD
    );


    // without light we would just get a black screen
    Ogre::Light *light = scnMgr->createLight("MainLight");
    light->setDiffuseColour(0.5, 0.5, 0.5);
    light->setSpecularColour(0.5, 0.5, 0.5);
    Ogre::SceneNode *lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    lightNode->setPosition(0, 10, 15);
    lightNode->attachObject(light);

    scnMgr->setAmbientLight(Ogre::ColourValue(0, 0, 0));
    scnMgr->setShadowTechnique(Ogre::ShadowTechnique::SHADOWTYPE_STENCIL_ADDITIVE);

    Ogre::Light *spotLight = scnMgr->createLight("SpotLight");
    spotLight->setDiffuseColour(0.5, 0.5, 0.5);
    spotLight->setSpecularColour(0.5, 0.5, 0.5);
    spotLight->setType(Ogre::Light::LT_SPOTLIGHT);
    Ogre::SceneNode *spotLightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    spotLightNode->attachObject(spotLight);
    spotLightNode->setDirection(-1, -1, 0);
    spotLightNode->setPosition(Ogre::Vector3(200, 200, 0));
    spotLight->setSpotlightRange(Ogre::Degree(35), Ogre::Degree(50));


    Ogre::Light *directionalLight = scnMgr->createLight("DirectionalLight");
    directionalLight->setType(Ogre::Light::LT_DIRECTIONAL);
    directionalLight->setDiffuseColour(Ogre::ColourValue(0.6, 0.2, 0.6));
    directionalLight->setSpecularColour(Ogre::ColourValue(0.6, 0.2, 0.6));
    Ogre::SceneNode *directionalLightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    directionalLightNode->attachObject(directionalLight);
    directionalLightNode->setDirection(Ogre::Vector3(0, -1, -1));
    directionalLightNode->setPosition(0, 100, 100);


    Ogre::Light *directionalLight2 = scnMgr->createLight("DirectionalLight2");
    directionalLight2->setType(Ogre::Light::LT_DIRECTIONAL);
    directionalLight2->setDiffuseColour(Ogre::ColourValue(0.2, 0.8, 0.6));
    directionalLight2->setSpecularColour(Ogre::ColourValue(0.2, 0.8, 0.6));
    Ogre::SceneNode *directionalLightNode2 = scnMgr->getRootSceneNode()->createChildSceneNode();
    directionalLightNode2->attachObject(directionalLight2);
    directionalLightNode2->setDirection(Ogre::Vector3(0, -1, 1));
    directionalLightNode2->setPosition(0, 100, -100);


    Ogre::Light *pointLight = scnMgr->createLight("PointLight");
    pointLight->setType(Ogre::Light::LT_POINT);
    pointLight->setDiffuseColour(0.3, 0.3, 0.3);
    pointLight->setSpecularColour(0.3, 0.3, 0.3);
    Ogre::SceneNode *pointLightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    pointLightNode->attachObject(pointLight);
    pointLightNode->setPosition(Ogre::Vector3(0, 150, 250));






    // also need to tell where we are
    Ogre::SceneNode *camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
//    camNode->setPosition(0, 0, 15);
//    camNode->lookAt(Ogre::Vector3(0, 0, -1), Ogre::Node::TS_PARENT);
    camNode->setPosition(200, 300, 400);
    camNode->lookAt(Ogre::Vector3(0, 0, 0), Ogre::Node::TransformSpace::TS_WORLD);

    // create the camera
    Ogre::Camera *cam = scnMgr->createCamera("myCam");
    cam->setNearClipDistance(5); // specific to this sample
    cam->setAutoAspectRatio(true);
    camNode->attachObject(cam);

    // https://stackoverflow.com/questions/23474492/orbit-camera-implementation-in-ogre
    // https://ogrecave.github.io/ogre/api/latest/class_ogre_bites_1_1_camera_man.html
    auto cameraMan = std::make_unique<OgreBites::CameraMan>(camNode);
    cameraMan->setStyle(OgreBites::CS_ORBIT);
//    cameraMan->setStyle(OgreBites::CS_FREELOOK);
//    cameraMan->setStyle(OgreBites::CS_MANUAL);

    // and tell it to render into the main window
    auto vp = ctx->getRenderWindow()->addViewport(cam);
    vp->setBackgroundColour(Ogre::ColourValue(1, 0, 1));


//    {
//        // finally something to render
//        Ogre::Entity *ent = scnMgr->createEntity("Sinbad.mesh");
//        Ogre::SceneNode *node = scnMgr->getRootSceneNode()->createChildSceneNode();
//        node->attachObject(ent);
//
//        auto b = dynamicsWorld->addRigidBody(
//                0,
//                ent,
//                Ogre::Bullet::ColliderType::CT_HULL,
//                boost::make_shared<Ogre::Bullet::DynamicsWorld::Bullet2OgreTracer>(
//                        ctx,
//                        root,
//                        scnMgr,
//                        ent
//                )
//        );
//    }
    {
        // https://ogrecave.github.io/ogre/api/latest/tut__lights_cameras_shadows.html
        Ogre::Entity *ninjaEntity = scnMgr->createEntity("ninja.mesh");
        ninjaEntity->setCastShadows(true);

        scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(ninjaEntity);

        auto b = dynamicsWorld->addRigidBody(
                0,
                ninjaEntity,
                Ogre::Bullet::ColliderType::CT_TRIMESH,
                boost::make_shared<Ogre::Bullet::DynamicsWorld::Bullet2OgreTracer>(
                        ctx,
                        root,
                        scnMgr,
                        ninjaEntity
                )
        );
    }

    Ogre::Plane plane(Ogre::Vector3::UNIT_Y,
    0);
    Ogre::MeshManager::getSingleton().createPlane(
            "ground", Ogre::RGN_DEFAULT,
            plane,
            1500, 1500, 20, 20,
            true,
            1, 5, 5,
            Ogre::Vector3::UNIT_Z);
    Ogre::Entity *groundEntity = scnMgr->createEntity("ground");
    scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(groundEntity);
    groundEntity->setCastShadows(false);
    // rockwall.tga
    groundEntity->setMaterialName("Rockwall");


//    {
//        createBoxMesh();
//
//        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
//                "ColourBoxMeshTest", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
//        material->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT);
//
//        Ogre::Entity *thisEntity = scnMgr->createEntity("cc", "ColourBoxMesh");
//        thisEntity->setMaterialName("ColourBoxMeshTest");
//        Ogre::SceneNode *thisSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
//        thisSceneNode->setPosition(-35, 0, 0);
////        thisSceneNode->rotate(Ogre::Vector3{0, 1, 0}, Ogre::Degree(90.0));
//        thisSceneNode->attachObject(thisEntity);
//
//    }

//    {
//        createBoxMesh2(scnMgr);
//
//        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
//                "ColourBoxMeshTest", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
//        material->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT);
//
//        Ogre::Entity *thisEntity = scnMgr->createEntity("cc", "ColourBoxMeshTest2");
//        thisEntity->setMaterialName("ColourBoxMeshTest");
//        Ogre::SceneNode *thisSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
//        thisSceneNode->setPosition(-35, 0, 0);
//        thisSceneNode->rotate(Ogre::Vector3{1, 1, 0}, Ogre::Degree(90.0));
//        thisSceneNode->attachObject(thisEntity);
//
//    }



//    {
//        auto info = createTtfMesh(scnMgr, ttfMeshFactory, L'中', 0.1);
//
//        if (!info) {
//            return -2;
//        }
//
//        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
//                "ColourTtfMeshTest", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
//        material->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT);
//
//        Ogre::Entity *thisEntity = scnMgr->createEntity(info->meshPtr);
//        thisEntity->setMaterialName("ColourBoxMeshTest");
//        Ogre::SceneNode *thisSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
//        Ogre::Vector3 scale{100, 100, 100};
//        std::cout << "getPos:" << info->info3D->getPos().toOgreVec3() << std::endl;
//        std::cout << "getSize:" << info->info3D->getSize().toOgreVec3() << std::endl;
//        std::cout << "getSize * scale:" << info->info3D->getSize().toOgreVec3() * scale << std::endl;
//        thisSceneNode->setPosition(
//                info->info3D->getPos().toOgreVec3()
//                * scale
//                + Ogre::Vector3{0, 0, 100}
//        );
//        thisSceneNode->setScale(scale);
//        thisSceneNode->showBoundingBox(true);
////        thisSceneNode->rotate(Ogre::Vector3{1, 0, 0}, Ogre::Degree(90.0));
//        thisSceneNode->createChildSceneNode()->attachObject(thisEntity);
//    }

    {
        std::vector<decltype(createTtfMesh(scnMgr, ttfMeshFactory, L'中', 0.1))> ttfMeshList1{
        };
        for (auto &s: std::wstring{L"中华人民共和国万岁"}) {
            ttfMeshList1.push_back(createTtfMesh(scnMgr, ttfMeshFactory, s, 0.1));
        }
        std::vector<decltype(createTtfMesh(scnMgr, ttfMeshFactory, L'中', 0.1))> ttfMeshList2{
        };
        for (auto &s: std::wstring{L"世界人民大团结万岁"}) {
            ttfMeshList2.push_back(createTtfMesh(scnMgr, ttfMeshFactory, s, 0.1));
        }
        std::vector<Ogre::Entity *> entityList;

        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
                "ColourTtfMeshTest2", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT);

        Ogre::SceneNode *thisSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        float scaleSize = 50;
        Ogre::Vector3 scale{scaleSize, scaleSize, scaleSize};

        Ogre::Vector3 sizeTotal1{0, 0, 0};
        Ogre::Vector3 basePos{0, 0, 0};
        for (auto &info: ttfMeshList1) {

            Ogre::Entity *thisEntity = scnMgr->createEntity(info->meshPtr);
            thisEntity->setMaterial(material);


            std::cout << "getPos:" << info->info3D->getPos().toOgreVec3() << std::endl;
            std::cout << "getSize:" << info->info3D->getSize().toOgreVec3() << std::endl;
            std::cout << "getSize * scale:" << info->info3D->getSize().toOgreVec3() * scale << std::endl;
            auto thisNode = thisSceneNode->createChildSceneNode(
                    Ogre::Vector3{sizeTotal1.x, 0, 0} + basePos
            );
            auto size = info->info3D->getSize().toOgreVec3();
            sizeTotal1 = Ogre::Vector3{sizeTotal1.x + size.x, std::max(sizeTotal1.y, size.y), sizeTotal1.z};
            thisNode->attachObject(thisEntity);

            auto b = dynamicsWorld->addRigidBody(
                    0,
                    thisEntity,
                    Ogre::Bullet::ColliderType::CT_TRIMESH,
                    boost::make_shared<Ogre::Bullet::DynamicsWorld::Bullet2OgreTracer>(
                            ctx,
                            root,
                            scnMgr,
                            thisEntity
                    )
            );

            entityList.push_back(thisEntity);
        }
        Ogre::Vector3 sizeTotal2{0, 0, 0};
        basePos = {0, -(sizeTotal1.y * 1.5f), 0};
        for (auto &info: ttfMeshList2) {

            Ogre::Entity *thisEntity = scnMgr->createEntity(info->meshPtr);
            thisEntity->setMaterial(material);


            std::cout << "getPos:" << info->info3D->getPos().toOgreVec3() << std::endl;
            std::cout << "getSize:" << info->info3D->getSize().toOgreVec3() << std::endl;
            std::cout << "getSize * scale:" << info->info3D->getSize().toOgreVec3() * scale << std::endl;
            auto thisNode = thisSceneNode->createChildSceneNode(
                    Ogre::Vector3{sizeTotal2.x, 0, 0} + basePos
            );
            auto size = info->info3D->getSize().toOgreVec3();
            sizeTotal2 = Ogre::Vector3{sizeTotal2.x + size.x, std::max(sizeTotal2.y, size.y), sizeTotal2.z};
            thisNode->attachObject(thisEntity);

            auto b = dynamicsWorld->addRigidBody(
                    0,
                    thisEntity,
                    Ogre::Bullet::ColliderType::CT_TRIMESH,
                    boost::make_shared<Ogre::Bullet::DynamicsWorld::Bullet2OgreTracer>(
                            ctx,
                            root,
                            scnMgr,
                            thisEntity
                    )
            );

            entityList.push_back(thisEntity);
        }
        Ogre::Vector3 sizeTotal{
                std::max(sizeTotal1.x, sizeTotal2.x),
                std::max(sizeTotal1.y, sizeTotal2.y + (-basePos.y)),
                std::max(sizeTotal1.z, sizeTotal2.z),
        };

        std::cout << "sizeTotal:" << sizeTotal << std::endl;
        thisSceneNode->setPosition(
                -Ogre::Vector3{sizeTotal.x, 0, 0} / 2 * scale
                + Ogre::Vector3{0, 20, 0}
                + Ogre::Vector3{0, +(-basePos.y) * scaleSize, 0}
                + Ogre::Vector3{0, 0, 100}
        );
        thisSceneNode->setScale(scale);
        thisSceneNode->rotate(Ogre::Vector3{1, 0, 0}, Ogre::Degree(-45.0));
//        thisSceneNode->showBoundingBox(true);

        const auto &memoryContainerManager_ = dynamicsWorld->getMemoryContainerManager();
        for (auto &p: entityList) {
            auto idAny = p->getUserObjectBindings().getUserAny("id_bullet");
            if (idAny.has_value()) {
                auto id = any_cast<int>(&idAny);
                if (id) {
                    auto b = memoryContainerManager_->getBody(*id);
                    if (b) {
//                        if (b->userPtr &&
//                            b->userPtr->typeName == Ogre::Bullet::DynamicsWorld::Bullet2OgreTracer::TypeNameTag) {
//                            auto t = dynamic_pointer_cast<Ogre::Bullet::DynamicsWorld::Bullet2OgreTracer>(b->userPtr);
//                            if (t) {
//                                t;
//                            }
//                        }

                        auto pn = p->getParentNode();
                        pn->convertLocalToWorldOrientation(pn->getOrientation());
                        pn->convertLocalToWorldPosition(pn->getPosition());
                        pn;
                        pn->getScale();

                        auto trans = btTransform(
                                Ogre::Bullet::convert(
                                        pn->convertLocalToWorldOrientation(Ogre::Quaternion::IDENTITY)
                                ),
                                Ogre::Bullet::convert(
                                        pn->convertLocalToWorldPosition(Ogre::Vector3::ZERO)
                                ));
//                        auto trans = btTransform(Ogre::Bullet::convert(p->getParentNode()->getOrientation()),
//                                                 Ogre::Bullet::convert(p->getParentNode()->getPosition()));
                        b->ptrRigidBody->setWorldTransform(trans);
                        b->ptrRigidBody->getCollisionShape()->setLocalScaling(
                                Ogre::Bullet::convert(
                                        thisSceneNode->getScale()
                                )
                        );
                        b->ptrRigidBody->activate(true);

                        std::cout << "(auto &p: entityList) update : " << p
                                  << " id :" << *id
                                  << " name : " << b->name
                                  << " getOrigin : "
                                  << "[" << trans.getOrigin().x()
                                  << "," << trans.getOrigin().y()
                                  << "," << trans.getOrigin().z() << "]"
                                  << " getRotation : "
                                  << "[" << trans.getRotation().x()
                                  << "," << trans.getRotation().y()
                                  << "," << trans.getRotation().z() << "]"
                                  << " getScale : "
                                  << "[" << thisSceneNode->getScale().x
                                  << "," << thisSceneNode->getScale().y
                                  << "," << thisSceneNode->getScale().z << "]"
                                  << std::endl;
                    }
                }
            } else {
                std::cout << "(auto &p: entityList) no id_bullet : " << p << std::endl;
            }
        }
    }


//    {
//        createColourCube();
//
//        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
//                "ColourTest", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
//        material->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT);
//
//        Ogre::Entity *thisEntity = scnMgr->createEntity("cc", "ColourCube");
//        thisEntity->setMaterialName("ColourTest");
//        Ogre::SceneNode *thisSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
//        thisSceneNode->setPosition(-35, 0, 0);
//        thisSceneNode->attachObject(thisEntity);
//    }

    {
        Ogre::SceneNode *mSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();


        auto mesh = createMeshObject(scnMgr,
                                     generator::SphereMesh{5.0, 6, 6},
                                     "TestSphereMesh");

        Ogre::Entity *mEntity = scnMgr->createEntity(mesh);
        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
                "ColourTest", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT);

        mEntity->setMaterial(material);
        mSceneNode->attachObject(mEntity);
        mSceneNode->setPosition(Ogre::Vector3(0, 200, 120));

//        Ogre::Real radius = mEntity->getBoundingRadius();
//        mSceneNode->scale(50 / radius, 50 / radius, 50 / radius);
//        mSceneNode->setScale(Ogre::Vector3(1.5, 1.5, 1.5)); // Radius, in theory.


        auto b = dynamicsWorld->addRigidBody(
                1,
                mEntity,
                Ogre::Bullet::ColliderType::CT_SPHERE,
                boost::make_shared<Ogre::Bullet::DynamicsWorld::Bullet2OgreTracer>(
                        ctx,
                        root,
                        scnMgr,
                        mEntity
                )
        );
    }


    auto imGuiOverlay = std::make_unique<Ogre::ImGuiOverlay>();

    // handle DPI scaling
    float vpScale = Ogre::OverlayManager::getSingleton().getPixelRatio();
    ImGui::GetIO().FontGlobalScale = std::round(vpScale); // default font does not work with fractional scaling
    ImGui::GetStyle().ScaleAllSizes(vpScale);

    imGuiOverlay->setZOrder(300);
    imGuiOverlay->show();
    // now owned by OverlayManager
    Ogre::OverlayManager::getSingleton().addOverlay(imGuiOverlay.release());

    // must call this to register ImGuiOverlay
    scnMgr->addRenderQueueListener(&Ogre::OverlaySystem::getSingleton());

    auto imGuiInputListener = std::make_unique<OgreBites::ImGuiInputListener>();
    ctx->addInputListener(&*imGuiInputListener);


    ImGuiDrawHandler imGuiDrawHandler;
    ctx->getRoot()->addFrameListener(&imGuiDrawHandler);

//    BulletHandler bulletHandler{scnMgr};
//    ctx->getRoot()->addFrameListener(&bulletHandler);

    // register for input events
    KeyHandler keyHandler;
    ctx->addInputListener(&keyHandler);
    ctx->addInputListener(&*cameraMan);

//    ctx->getRoot()->startRendering();

    Ogre::Root::getSingleton().getRenderSystem()->_initRenderTargets();
    ctx->getRoot()->clearEventTimes();

    while (!ctx->getRoot()->endRenderingQueued()) {
        dynamicsWorld->stepSimulation(1 / 60.f);
        auto dirtyIds = dynamicsWorld->getDirtyBodyIds();
        for (const auto &a: dirtyIds) {
            auto bp = bulletMemoryContainerManager->getBody(a.first);
            if (bp) {
                if (bp->userPtr) {
//                    std::cout << "bp->userPtr->typeName " << bp->userPtr->typeName << std::endl;
                    if (bp->userPtr->typeName == Ogre::Bullet::DynamicsWorld::Bullet2OgreTracer::TypeNameTag) {
                        auto tr = dynamic_pointer_cast<Ogre::Bullet::DynamicsWorld::Bullet2OgreTracer>(bp->userPtr);
                        Ogre::Bullet::RigidBodyState::UpdateNodeTransform(*(tr->sceneNode), a.second);
                    }
                }
            }
        }
        ctx->getRoot()->renderOneFrame();
        dynamicsWorld->updateDebugDrawWorld();
//        if (ctx->getRenderWindow()->isActive() || ctx->getRenderWindow()->isVisible()) {
//            if (!ctx->getRoot()->renderOneFrame()) {
//                break;
//            }
//        }
    }


    // must destroy before ctx, and before bulletMemoryContainerManager
    dynamicsWorld.reset();
    // must destroy before BulletMemoryPool::gpMemoryPoolManager
    bulletMemoryContainerManager.reset();

    {
        std::cout << "gpMemoryPoolManager leak items "
                  << BulletMemoryPool::gpMemoryPoolManager->_getMemoryPoolRef().size()
                  << std::endl;
        size_t s = 0;
        for (const auto &n: BulletMemoryPool::gpMemoryPoolManager->_getMemoryPoolRef()) {
            s += n.second.size;
        }
        std::cout << "gpMemoryPoolManager leak size "
                  << s
                  << std::endl;
    }
    // after destroy gpMemoryPoolManager , never call Bullet again
    BulletMemoryPool::gpMemoryPoolManager.reset();


    ctx->closeApp();


    return 0;
}
