using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;
using System.Linq;
using Assets.script;
using UnityEngine.Rendering;

public class GPUPBD : MonoBehaviour
{
    public enum MyModel
    {
        IcoSphere_low,
        Torus,
        Bunny,
        Armadillo,
    };

    [Header("3D model")]
    public MyModel model;
    [HideInInspector]
    private string modelName;

    [Header("Obj Parameters")]
    public int numberOfObjects = 1;
    public float invMass = 1.0f;
    public float dt = 0.01f; // have to devide by 20
    public Vector3 gravity = new Vector3(0f, -9.81f, 0f);
    public int iteration = 5;
    public float convergence_factor = 1.5f;

    [Header("Distance Constrinat Parameters")]
    public float stretchStiffness = 1.0f;
    public float compressStiffness = 1.0f;
    [Header("Bending Constrinat Parameters")]
    public float bendingStiffness = 1.0f;
    [Header("Volume  Constrinat Parameters")]
    public float volumeStiffness = 1.0f;

    [Header("Collision")]
    public GameObject floor;

    [Header("Triangle Intersections")]
    public bool calculateCollision = false;
    public bool printLog = false;

    [Header("Label Data")]
    public bool drawFPS;
    public bool renderVolumeText;
    public string Text;
    public int xOffset;
    public int yOffset;
    public int fontSize;
    public Color textColor = Color.white;
    private Rect rectPos;
    private Color color = Color.black;

    [Header("Volume Data")]
    public bool writeVolumeToFile;
    public bool writeImageFrame;
    public int maxFrameNum;
    public string directory = "";
    public string volumeFileName = "";

    [HideInInspector]
    private int nodeCount;
    private int springCount;
    private int triCount; // size of triangle
    private int tetCount;
    private int bendingCount;

    //main  property
    //list position
    Vector3[] Positions;
    Vector3[] ProjectPositions;
    Vector3[] WorldPositions;
    Vector3[] Velocities;
    Vector3[] Responses;
    Vector3[] Normals;
    Vector3[] CollisionDirections;
    List<Spring> distanceConstraints = new List<Spring>();
    List<Triangle> triangles = new List<Triangle>();
    List<Tetrahedron> tetrahedrons = new List<Tetrahedron>();
    List<Bending> bendingConstraints = new List<Bending>();
    //for render
    ComputeBuffer vertsBuff = null;
    ComputeBuffer triBuffer = null;

    //for compute shader
    private ComputeBuffer positionsBuffer;
    private ComputeBuffer projectedPositionsBuffer;
    private ComputeBuffer velocitiesBuffer;

    private ComputeBuffer triangleBuffer;
    private ComputeBuffer triangleIndicesBuffer;

    private ComputeBuffer triBuffer2;

    private ComputeBuffer deltaPositionsBuffer;
    private ComputeBuffer deltaPositionsIntBuffer;
    private ComputeBuffer deltaCounterBuffer;
    private ComputeBuffer directionIntBuffer;
    private ComputeBuffer directionCounterBuffer;

    private ComputeBuffer objVolumeBuffer;

    private ComputeBuffer distanceConstraintsBuffer;
    private ComputeBuffer bendingConstraintsBuffer;
    private ComputeBuffer tetVolConstraintsBuffer;

    //kerne id (might not use all currently)
    //private int applyExternalResponsesKernel;
    //private int dampVelocitiesKernel;
    private int applyExplicitEulerKernel;
    private int floorCollisionKernel;


    private int satisfyDistanceConstraintKernel;
    private int satisfyBendingConstraintKernel;
    private int satisfyTetVolConstraintKernel;

    private int averageConstraintDeltasKernel;
    private int updatePositionsKernel;

    private int computeObjVolumeKernel;   // for compute object's volume

    private int computeVerticesNormal; // for rendering purpose 

    private int computeCollisionHandling; // for collision detection
    private int computeCollisionResponse; // for collision response

    [Header("Rendering Paramenter")]
    public ComputeShader computeShader;
    public ComputeShader collisionComputeShader;
    public Shader renderingShader;
    public Color matColor;

    [HideInInspector]
    private Material material;
    private ComputeShader computeShaderobj;
    struct vertData
    {
        public Vector3 pos;
        public Vector2 uvs;
        public Vector3 norms;
    };
    int[] triArray;
    vertData[] vDataArray;
    private static GameObject obj;

    float totalVolume; //
    int frame = 0; //number ot time frame
    float[] volumeDataGPU = new float[1]; //use to get data from GPU
    Int3Struct[] directionDataGPU = new Int3Struct[1]; //use to get data from GPU

    [HideInInspector]
    List<string[]> tableData = new List<string[]>();


    void SelectModelName()
    {
        switch (model)
        {
            case MyModel.IcoSphere_low: modelName = "icosphere_low.1"; break;
            case MyModel.Torus: modelName = "torus.1"; break;
            case MyModel.Bunny: modelName = "bunny.1"; break;
            case MyModel.Armadillo: modelName = "Armadillo.1"; break;
        }
    }

    void addBendingConstraint()
    {
        Dictionary<Edge, List<Triangle>> wingEdges = new Dictionary<Edge, List<Triangle>>(new EdgeComparer());

        // map edges to all of the faces to which they are connected
        foreach (Triangle tri in triangles)
        {
            Edge e1 = new Edge(tri.vertices[0], tri.vertices[1]);
            if (wingEdges.ContainsKey(e1) && !wingEdges[e1].Contains(tri))
            {
                wingEdges[e1].Add(tri);
            }
            else
            {
                List<Triangle> tris = new List<Triangle>();
                tris.Add(tri);
                wingEdges.Add(e1, tris);
            }

            Edge e2 = new Edge(tri.vertices[0], tri.vertices[2]);
            if (wingEdges.ContainsKey(e2) && !wingEdges[e2].Contains(tri))
            {
                wingEdges[e2].Add(tri);
            }
            else
            {
                List<Triangle> tris = new List<Triangle>();
                tris.Add(tri);
                wingEdges.Add(e2, tris);
            }

            Edge e3 = new Edge(tri.vertices[1], tri.vertices[2]);
            if (wingEdges.ContainsKey(e3) && !wingEdges[e3].Contains(tri))
            {
                wingEdges[e3].Add(tri);
            }
            else
            {
                List<Triangle> tris = new List<Triangle>();
                tris.Add(tri);
                wingEdges.Add(e3, tris);
            }
        }

        // wingEdges are edges with 2 occurences,
        // so we need to remove the lower frequency ones
        List<Edge> keyList = wingEdges.Keys.ToList();
        foreach (Edge e in keyList)
        {
            if (wingEdges[e].Count < 2)
            {
                wingEdges.Remove(e);
            }
        }

        bendingCount = wingEdges.Count;

        foreach (Edge wingEdge in wingEdges.Keys)
        {
            /* wingEdges are indexed like in the Bridson,
                * Simulation of Clothing with Folds and Wrinkles paper
                *    3
                *    ^
                * 0  |  1
                *    2
                */

            int[] indices = new int[4];
            indices[2] = wingEdge.startIndex;
            indices[3] = wingEdge.endIndex;

            int b = 0;
            foreach (Triangle tri in wingEdges[wingEdge])
            {
                for (int i = 0; i < 3; i++)
                {
                    int point = tri.vertices[i];
                    if (point != indices[2] && point != indices[3])
                    {
                        //tri #1
                        if (b == 0)
                        {
                            indices[0] = point;
                            break;
                        }
                        //tri #2
                        else if (b == 1)
                        {
                            indices[1] = point;
                            break;
                        }
                    }
                }
                b++;
            }
            Vector3 p0 = Positions[indices[0]];
            Vector3 p1 = Positions[indices[1]];
            Vector3 p2 = Positions[indices[2]];
            Vector3 p3 = Positions[indices[3]];

            Vector3 n1 = (Vector3.Cross(p2 - p0, p3 - p0)).normalized;
            Vector3 n2 = (Vector3.Cross(p3 - p1, p2 - p1)).normalized;

            float d = Vector3.Dot(n1, n2);
            d = Mathf.Clamp(d, -1.0f, 1.0f);

            Bending bending = new Bending();
            bending.index0 = indices[0];
            bending.index1 = indices[1];
            bending.index2 = indices[2];
            bending.index3 = indices[3];
            bending.restAngle = Mathf.Acos(d);

            bendingConstraints.Add(bending);
        }
    }

    void setupMeshData(int number)
    {
        //print(Application.dataPath);
        string filePath = Application.dataPath + "/TetGen-Model/";
        LoadTetModel.LoadData(filePath + modelName, gameObject);

        var _Positions = LoadTetModel.positions.ToArray();
        var _triangles = LoadTetModel.triangles;
        var _distanceConstraints = LoadTetModel.springs;
        var _triArray = LoadTetModel.triangleArr.ToArray();
        var _tetrahedrons = LoadTetModel.tetrahedrons;
        var _bendingConstraints = LoadTetModel.bendings;

        Positions = new Vector3[number * LoadTetModel.positions.Count];
        triangles = new List<Triangle>(new Triangle[number * LoadTetModel.triangles.Count]);
        distanceConstraints = new List<Spring>(new Spring[number * LoadTetModel.springs.Count]);
        triArray = new int[number * LoadTetModel.triangleArr.Count];
        tetrahedrons = new List<Tetrahedron>(new Tetrahedron[number * LoadTetModel.tetrahedrons.Count]);
        bendingConstraints = new List<Bending>(new Bending[number * LoadTetModel.bendings.Count]);

        float ranges = 20.0f;
        string data = "";
        for (int i = 0; i < number; i++)
        {
            int PosOffset = i * LoadTetModel.positions.Count;
            Vector3 Offset = new Vector3(UnityEngine.Random.Range(-ranges, ranges), 5.0f + (i * 7.0f), UnityEngine.Random.Range(-ranges, ranges));
            for (int j = 0; j < LoadTetModel.positions.Count; j++)
            {
                Positions[j + PosOffset] = _Positions[j] + Offset;
            }
            int TriOffset = i * LoadTetModel.triangles.Count;
            for (int j = 0; j < LoadTetModel.triangles.Count; j++)
            {
                var t = _triangles[j];
                triangles[j + TriOffset] = new Triangle(t.vertices[0] + PosOffset, t.vertices[1] + PosOffset, t.vertices[2] + PosOffset);
                data += "[" + (j + TriOffset) + "]" + triangles[j + TriOffset].vertices[0] + "," + triangles[j + TriOffset].vertices[1] + "," + triangles[j + TriOffset].vertices[2] + "\r\n";
            }
            int TriArrOffset = i * LoadTetModel.triangleArr.Count;
            for (int j = 0; j < LoadTetModel.triangleArr.Count; j++)
            {
                triArray[j + TriArrOffset] = _triArray[j] + PosOffset;
            }
            int TetraOffset = i * LoadTetModel.tetrahedrons.Count;
            for (int j = 0; j < LoadTetModel.tetrahedrons.Count; j++)
            {
                Tetrahedron oldTetra = _tetrahedrons[j];
                Tetrahedron newTetra = new Tetrahedron(
                    oldTetra.i1 + PosOffset,
                    oldTetra.i2 + PosOffset,
                    oldTetra.i3 + PosOffset,
                    oldTetra.i4 + PosOffset,
                    oldTetra.RestVolume);
                tetrahedrons[j + TetraOffset] = newTetra;
            }
            int DistConstraintOffset = i * LoadTetModel.springs.Count;
            for (int j = 0; j < LoadTetModel.springs.Count; j++)
            {
                Spring oldSpring = _distanceConstraints[j];
                Spring newSpring = new Spring(
                    oldSpring.i1 + PosOffset,
                    oldSpring.i2 + PosOffset,
                    oldSpring.RestLength);
                distanceConstraints[j + DistConstraintOffset] = newSpring;
            }
            int bendingOffset = i * LoadTetModel.bendings.Count;
            for (int j = 0; j < LoadTetModel.bendings.Count; j++)
            {
                Bending oldBending = _bendingConstraints[j];
                Bending newBending = new Bending();
                newBending.index0 = oldBending.index0 + PosOffset;
                newBending.index1 = oldBending.index1 + PosOffset;
                newBending.index2 = oldBending.index2 + PosOffset;
                newBending.index3 = oldBending.index3 + PosOffset;
                newBending.restAngle = oldBending.restAngle;
                bendingConstraints[j + bendingOffset] = newBending;
            }
        }

        Debug.Log(data);

        nodeCount = Positions.Length;
        springCount = distanceConstraints.Count;
        triCount = triangles.Count; //
        tetCount = tetrahedrons.Count;
        bendingCount = bendingConstraints.Count;

        print("node count: " + nodeCount);
        print("stretch constraint: " + springCount);
        print("bending constraint: " + bendingCount);
        print("volume constraint: " + tetCount);

        WorldPositions = new Vector3[nodeCount];
        ProjectPositions = new Vector3[nodeCount];
        Velocities = new Vector3[nodeCount];
        Responses = new Vector3[nodeCount];
        WorldPositions.Initialize();
        Velocities.Initialize();
        Responses.Initialize();

        vDataArray = new vertData[nodeCount];

        for (int i = 0; i < nodeCount; i++)
        {
            vDataArray[i] = new vertData();
            vDataArray[i].pos = Positions[i];
            vDataArray[i].norms = Vector3.zero;
            vDataArray[i].uvs = Vector3.zero;
        }

        int triBuffStride = sizeof(int);
        triBuffer = new ComputeBuffer(triArray.Length,
            triBuffStride, ComputeBufferType.Default);

        int vertsBuffstride = 8 * sizeof(float);
        vertsBuff = new ComputeBuffer(vDataArray.Length,
            vertsBuffstride, ComputeBufferType.Default);

        LoadTetModel.ClearData();
    }
    private void setupShader()
    {
        material.SetBuffer(Shader.PropertyToID("vertsBuff"), vertsBuff);
        material.SetBuffer(Shader.PropertyToID("triBuff"), triBuffer);
    }
    private void setBuffData()
    {
        vertsBuff.SetData(vDataArray);
        triBuffer.SetData(triArray);

        Vector3 translation = transform.position;
        Vector3 scale = this.transform.localScale;
        Quaternion rotationeuler = transform.rotation;
        Matrix4x4 trs = Matrix4x4.TRS(translation, rotationeuler, scale);
        material.SetMatrix("TRSMatrix", trs);
        material.SetMatrix("invTRSMatrix", trs.inverse);
    }

    private void setupComputeBuffer()
    {
        positionsBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        positionsBuffer.SetData(Positions);

        velocitiesBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        velocitiesBuffer.SetData(Velocities);

        projectedPositionsBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        projectedPositionsBuffer.SetData(Positions);

        UInt3Struct[] deltaPosUintArray = new UInt3Struct[nodeCount];
        deltaPosUintArray.Initialize();

        Int3Struct[] directionIntArray = new Int3Struct[nodeCount];
        directionIntArray.Initialize();

        directionDataGPU = new Int3Struct[nodeCount];

        Vector3[] deltaPositionArray = new Vector3[nodeCount];
        deltaPositionArray.Initialize();

        int[] deltaCounterArray = new int[nodeCount];
        deltaCounterArray.Initialize();

        int[] directionCounterArray = new int[nodeCount];
        directionCounterArray.Initialize();

        deltaPositionsBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        deltaPositionsBuffer.SetData(deltaPositionArray);

        deltaPositionsIntBuffer = new ComputeBuffer(nodeCount, sizeof(uint) * 3);
        deltaPositionsIntBuffer.SetData(deltaPosUintArray);

        directionIntBuffer = new ComputeBuffer(nodeCount, sizeof(int) * 3);
        directionIntBuffer.SetData(directionIntArray);

        directionCounterBuffer = new ComputeBuffer(nodeCount, sizeof(int));
        directionCounterBuffer.SetData(directionCounterArray);

        deltaCounterBuffer = new ComputeBuffer(nodeCount, sizeof(int));
        deltaCounterBuffer.SetData(deltaCounterArray);

        List<MTriangle> initTriangle = new List<MTriangle>();  //list of triangle cooresponding to node 
        List<int> initTrianglePtr = new List<int>(); //contain a group of affectd triangle to node
        //initTrianglePtr.Add(0);

        MTriangle[] _tris = new MTriangle[triCount];

        for (int i = 0; i < triangles.Count; i++)
        {
            _tris[i].v0 = triangles[i].vertices[0];
            _tris[i].v1 = triangles[i].vertices[1];
            _tris[i].v2 = triangles[i].vertices[2];
        }

        // for (int i = 0; i < nodeCount; i++)
        // {
        //     foreach (Triangle tri in triangles)
        //     {
        //         if (tri.vertices[0] == i || tri.vertices[1] == i || tri.vertices[2] == i)
        //         {
        //             MTriangle tmpTri = new MTriangle();
        //             tmpTri.v0 = tri.vertices[0];
        //             tmpTri.v1 = tri.vertices[1];
        //             tmpTri.v2 = tri.vertices[2];
        //             initTriangle.Add(tmpTri);
        //         }
        //     }
        //     initTrianglePtr.Add(initTriangle.Count);
        // }        

        Dictionary<int, List<int>> nodeTriangles = new Dictionary<int, List<int>>();
        for (int triIndex = 0; triIndex < triangles.Count; triIndex++)
        {
            Triangle tri = triangles[triIndex];
            for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
            {
                int vertex = tri.vertices[vertexIndex];
                if (!nodeTriangles.ContainsKey(vertex))
                {
                    nodeTriangles[vertex] = new List<int>();
                }
                nodeTriangles[vertex].Add(triIndex);
            }
        }
        initTrianglePtr.Add(0);
        for (int i = 0; i < nodeCount; i++)
        {
            if (nodeTriangles.TryGetValue(i, out List<int> triangleIndexes))
            {
                foreach (int triIndex in triangleIndexes)
                {
                    Triangle tri = triangles[triIndex];
                    MTriangle tmpTri = new MTriangle { v0 = tri.vertices[0], v1 = tri.vertices[1], v2 = tri.vertices[2] };
                    initTriangle.Add(tmpTri);
                }
            }
            initTrianglePtr.Add(initTriangle.Count);
        }

        //print(initTrianglePtr);

        triangleBuffer = new ComputeBuffer(initTriangle.Count, (sizeof(int) * 3));
        triangleBuffer.SetData(initTriangle.ToArray());

        triBuffer2 = new ComputeBuffer(_tris.Length, (sizeof(int) * 3));
        triBuffer2.SetData(_tris);

        triangleIndicesBuffer = new ComputeBuffer(initTrianglePtr.Count, sizeof(int));
        triangleIndicesBuffer.SetData(initTrianglePtr.ToArray());


        distanceConstraintsBuffer = new ComputeBuffer(springCount, sizeof(float) + sizeof(int) * 2);
        distanceConstraintsBuffer.SetData(distanceConstraints.ToArray());

        bendingConstraintsBuffer = new ComputeBuffer(bendingCount, sizeof(float) + sizeof(int) * 4);
        bendingConstraintsBuffer.SetData(bendingConstraints.ToArray());

        tetVolConstraintsBuffer = new ComputeBuffer(tetCount, sizeof(float) + sizeof(int) * 4);
        tetVolConstraintsBuffer.SetData(tetrahedrons.ToArray());

        uint[] initUint = new uint[1];
        initUint.Initialize();
        objVolumeBuffer = new ComputeBuffer(1, sizeof(uint));
        objVolumeBuffer.SetData(initUint);
    }


    float computeObjectVolume()
    {
        //made by sum of all tetra
        float volume = 0.0f;
        foreach (Tetrahedron tet in tetrahedrons)
        {
            volume += tet.RestVolume;
        }
        return volume;
    }
    private void setupKernel()
    {
        applyExplicitEulerKernel = computeShaderobj.FindKernel("applyExplicitEulerKernel");

        floorCollisionKernel = computeShaderobj.FindKernel("floorCollisionKernel");
        //for solving all constraint at once
        //satisfyPointConstraintsKernel = computeShaderobj.FindKernel("projectConstraintDeltasKernel");
        //satisfySphereCollisionsKernel = computeShaderobj.FindKernel("projectConstraintDeltasKernel");
        //satisfyCubeCollisionsKernel = computeShaderobj.FindKernel("projectConstraintDeltasKernel");
        //for solving constrint one-by-one
        satisfyDistanceConstraintKernel = computeShaderobj.FindKernel("satisfyDistanceConstraintKernel");
        averageConstraintDeltasKernel = computeShaderobj.FindKernel("averageConstraintDeltasKernel");

        satisfyBendingConstraintKernel = computeShaderobj.FindKernel("satisfyBendingConstraintKernel");
        satisfyTetVolConstraintKernel = computeShaderobj.FindKernel("satisfyTetVolConstraintKernel");

        //update position
        updatePositionsKernel = computeShaderobj.FindKernel("updatePositionsKernel");
        //object volume
        computeObjVolumeKernel = computeShaderobj.FindKernel("computeObjVolumeKernel");
        //for rendering
        computeVerticesNormal = computeShaderobj.FindKernel("computeVerticesNormal");

        computeCollisionHandling = collisionComputeShader.FindKernel("CollisionHandling");

        computeCollisionResponse = collisionComputeShader.FindKernel("CollisionResponse");

    }
    private void setupComputeShader()
    {
        //send uniform data for kernels in compute shader
        computeShaderobj.SetInt("nodeCount", nodeCount);
        computeShaderobj.SetInt("springCount", springCount);
        computeShaderobj.SetInt("triCount", triCount);
        computeShaderobj.SetInt("tetCount", tetCount);
        computeShaderobj.SetInt("bendingCount", bendingCount);
        collisionComputeShader.SetInt("triCount", triCount / numberOfObjects);
        Debug.Log("triCount per each Object : " + triCount / numberOfObjects);

        computeShaderobj.SetFloat("dt", dt);
        computeShaderobj.SetFloat("invMass", invMass);
        computeShaderobj.SetFloat("stretchStiffness", stretchStiffness);
        computeShaderobj.SetFloat("compressStiffness", compressStiffness);
        computeShaderobj.SetFloat("bendingStiffness", bendingStiffness);
        computeShaderobj.SetFloat("tetVolStiffness", volumeStiffness);
        computeShaderobj.SetFloat("convergence_factor", convergence_factor);

        computeShaderobj.SetVector("gravity", gravity);

        // bind buffer data to each kernel

        //Kernel #1 add force & apply euler
        computeShaderobj.SetBuffer(applyExplicitEulerKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(applyExplicitEulerKernel, "Positions", positionsBuffer);
        computeShaderobj.SetBuffer(applyExplicitEulerKernel, "ProjectedPositions", projectedPositionsBuffer);

        //Kernel #2
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "deltaPos", deltaPositionsBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "deltaPosAsInt", deltaPositionsIntBuffer);   //for find the correct project position
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "deltaCount", deltaCounterBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "ProjectedPositions", projectedPositionsBuffer);
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "distanceConstraints", distanceConstraintsBuffer);
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "Positions", positionsBuffer);

        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "deltaPos", deltaPositionsBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "deltaPosAsInt", deltaPositionsIntBuffer);   //for find the correct project position
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "deltaCount", deltaCounterBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "ProjectedPositions", projectedPositionsBuffer);
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "bendingConstraints", bendingConstraintsBuffer);
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "Positions", positionsBuffer);

        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "deltaPos", deltaPositionsBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "deltaPosAsInt", deltaPositionsIntBuffer);   //for find the correct project position
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "deltaCount", deltaCounterBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "ProjectedPositions", projectedPositionsBuffer);
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "tetVolumeConstraints", tetVolConstraintsBuffer);
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "Positions", positionsBuffer);

        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "deltaPos", deltaPositionsBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "deltaPosAsInt", deltaPositionsIntBuffer);   //for find the correct project position
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "deltaCount", deltaCounterBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "ProjectedPositions", projectedPositionsBuffer);
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "distanceConstraints", distanceConstraintsBuffer);
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "Positions", positionsBuffer);

        computeShaderobj.SetBuffer(floorCollisionKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(floorCollisionKernel, "Positions", positionsBuffer);
        computeShaderobj.SetBuffer(floorCollisionKernel, "ProjectedPositions", projectedPositionsBuffer);
        //Kernel  update position
        //computeShaderobj.SetBuffer(applyExplicitEulerKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(updatePositionsKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(updatePositionsKernel, "Positions", positionsBuffer);
        computeShaderobj.SetBuffer(updatePositionsKernel, "ProjectedPositions", projectedPositionsBuffer);
        computeShaderobj.SetBuffer(updatePositionsKernel, "vertsBuff", vertsBuff); //passing to rendering

        computeShaderobj.SetBuffer(computeObjVolumeKernel, "objVolume", objVolumeBuffer);
        computeShaderobj.SetBuffer(computeObjVolumeKernel, "Positions", positionsBuffer);
        computeShaderobj.SetBuffer(computeObjVolumeKernel, "tetVolumeConstraints", tetVolConstraintsBuffer);

        //kernel compute vertices normal
        computeShaderobj.SetBuffer(computeVerticesNormal, "Positions", positionsBuffer);
        computeShaderobj.SetBuffer(computeVerticesNormal, "Triangles", triangleBuffer);
        computeShaderobj.SetBuffer(computeVerticesNormal, "TrianglePtr", triangleIndicesBuffer);
        computeShaderobj.SetBuffer(computeVerticesNormal, "vertsBuff", vertsBuff); //passing to rendering
        computeShaderobj.SetBuffer(computeVerticesNormal, "objVolume", objVolumeBuffer);

        collisionComputeShader.SetBuffer(computeCollisionHandling, "positions", positionsBuffer);
        collisionComputeShader.SetBuffer(computeCollisionHandling, "velocities", velocitiesBuffer);
        collisionComputeShader.SetBuffer(computeCollisionHandling, "triangles", triBuffer2);
        collisionComputeShader.SetBuffer(computeCollisionHandling, "directions", directionIntBuffer);
        collisionComputeShader.SetBuffer(computeCollisionHandling, "directionCount", directionCounterBuffer);

        collisionComputeShader.SetBuffer(computeCollisionResponse, "positions", positionsBuffer);
        collisionComputeShader.SetBuffer(computeCollisionResponse, "velocities", velocitiesBuffer);
        collisionComputeShader.SetBuffer(computeCollisionResponse, "triangles", triBuffer2);
        collisionComputeShader.SetBuffer(computeCollisionResponse, "directions", directionIntBuffer);
        collisionComputeShader.SetBuffer(computeCollisionResponse, "directionCount", directionCounterBuffer);
    }

    void setup()
    {
        material = new Material(renderingShader); // new material for difference object
        material.color = matColor; //set color to material
        computeShaderobj = Instantiate(computeShader); // to instantiate the compute shader to be use with multiple object
        double lastInterval = Time.realtimeSinceStartup;
        SelectModelName();
        Debug.Log("SelectModelName: " + (Time.realtimeSinceStartup - lastInterval));
        lastInterval = Time.realtimeSinceStartup;
        setupMeshData(numberOfObjects);
        Debug.Log("setupMeshData: " + (Time.realtimeSinceStartup - lastInterval));
        lastInterval = Time.realtimeSinceStartup;
        setupShader();
        Debug.Log("setupShader: " + (Time.realtimeSinceStartup - lastInterval));
        lastInterval = Time.realtimeSinceStartup;
        setBuffData();
        Debug.Log("setBuffData: " + (Time.realtimeSinceStartup - lastInterval));
        lastInterval = Time.realtimeSinceStartup;
        setupComputeBuffer();
        Debug.Log("setupComputeBuffer: " + (Time.realtimeSinceStartup - lastInterval));
        lastInterval = Time.realtimeSinceStartup;
        setupKernel();
        Debug.Log("setupKernel: " + (Time.realtimeSinceStartup - lastInterval));
        lastInterval = Time.realtimeSinceStartup;
        setupComputeShader();
        Debug.Log("setupComputeShader: " + (Time.realtimeSinceStartup - lastInterval));
        lastInterval = Time.realtimeSinceStartup;

        totalVolume = computeObjectVolume();
    }
    void Start()
    {
        setup();
    }
    // Update is called once per frame
    void dispatchComputeShader()
    {
        ////update uniform data and GPU buffer here
        ////PBD algorithm
        computeShaderobj.Dispatch(applyExplicitEulerKernel, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);

        for (int i = 0; i < 1; i++)
        {
            if (calculateCollision)
            {
                collisionComputeShader.Dispatch(computeCollisionHandling, (int)Mathf.Ceil(triCount / 16.0f), (int)Mathf.Ceil(triCount / 16.0f), 1);

                if (printLog)
                {
                    directionIntBuffer.GetData(directionDataGPU);

                    for (int j = 0; j < directionDataGPU.Length; j++)
                    {
                        if (directionDataGPU[j].deltaXInt == 0 && directionDataGPU[j].deltaYInt == 0 && directionDataGPU[j].deltaZInt == 0) { continue; }
                        Debug.Log(j + "=" + directionDataGPU[j].deltaXInt + "," + directionDataGPU[j].deltaYInt + "," + directionDataGPU[j].deltaZInt);
                    }
                }

                collisionComputeShader.Dispatch(computeCollisionResponse, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
            }
        }

        ////damp velocity() here
        for (int i = 0; i < iteration; i++)
        {


            //solving constraint using avaerage jacobi style
            //convergence rate slower that Gauss–Seidel method implement on CPU method
            computeShaderobj.Dispatch(satisfyDistanceConstraintKernel, (int)Mathf.Ceil(springCount / 1024.0f), 1, 1);
            computeShaderobj.Dispatch(satisfyBendingConstraintKernel, (int)Mathf.Ceil(bendingCount / 1024.0f), 1, 1);
            computeShaderobj.Dispatch(satisfyTetVolConstraintKernel, (int)Mathf.Ceil(tetCount / 1024.0f), 1, 1);
            computeShaderobj.Dispatch(averageConstraintDeltasKernel, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);

            computeShaderobj.SetFloat("floorCoordY", (floor.transform.position).y);
            computeShaderobj.SetFloat("floorCoordX1", -25);
            computeShaderobj.SetFloat("floorCoordX2", 25);
            computeShaderobj.SetFloat("floorCoordZ1", -25);
            computeShaderobj.SetFloat("floorCoordZ2", 25);
            computeShaderobj.Dispatch(floorCollisionKernel, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
        }
        computeShaderobj.Dispatch(updatePositionsKernel, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);

        //compute object volume
        if (renderVolumeText)
        {
            computeShaderobj.Dispatch(computeObjVolumeKernel, (int)Mathf.Ceil(tetCount / 1024.0f), 1, 1);
            objVolumeBuffer.GetData(volumeDataGPU);
        }
        //compute normal for rendering
        computeShaderobj.Dispatch(computeVerticesNormal, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
    }
    void renderObject()
    {
        Bounds bounds = new Bounds(Vector3.zero, Vector3.one * 10000);
        material.SetPass(0);
        Graphics.DrawProcedural(material, bounds, MeshTopology.Triangles, triArray.Length,
            1, null, null, ShadowCastingMode.On, true, gameObject.layer);

    }
    void Update()
    {

        dispatchComputeShader();
        renderObject();

        if (writeVolumeToFile)
        {
            buildDataPerRow(frame, volumeDataGPU[0]);
            if (frame == maxFrameNum - 1)
            {
                //volume file name refer to prefix of the output file or method used
                writeTableData(volumeFileName + "_" + modelName);
                print("write Done");
                UnityEditor.EditorApplication.isPlaying = false;
            }
        }
        frame++;
    }
    void buildDataPerRow(int frame_num, float currVolume)
    {
        List<string> rowData = new List<string>();
        rowData.Add(frame.ToString());

        float Vpercentage = (currVolume / totalVolume) * 100.0f;
        rowData.Add(Vpercentage.ToString());
        tableData.Add(rowData.ToArray());
    }
    void writeTableData(string fileName)
    {
        string[][] output = new string[tableData.Count][];
        for (int i = 0; i < output.Length; i++)
        {
            output[i] = tableData[i];
        }
        int length = output.GetLength(0);
        string delimiter = ",";
        StringBuilder sb = new StringBuilder();

        for (int index = 0; index < length; index++)
            sb.AppendLine(string.Join(delimiter, output[index]));
        string filePath = directory + fileName + ".csv";



        StreamWriter outStream = System.IO.File.CreateText(filePath);
        outStream.WriteLine(sb);
        outStream.Close();
        tableData.Clear();
    }
    private void OnGUI()
    {
        if (renderVolumeText)
        {
            int w = Screen.width, h = Screen.height;
            GUIStyle style = new GUIStyle();
            rectPos = new Rect(0 + xOffset, yOffset, w, h * 2 / 100);
            Rect rect = rectPos;
            style.alignment = TextAnchor.UpperLeft;
            style.fontSize = h * 2 / 50;
            Color col;
            string htmlValue = "#FFED00";
            if (ColorUtility.TryParseHtmlString(htmlValue, out col))
                style.normal.textColor = col;

            //get volume data;

            float currVolume = volumeDataGPU[0];
            //currVolume = computeSurfaceVolume();

            float vLost;
            if (totalVolume == 0.0f)
            {
                vLost = 0.0f;
            }
            else
            {
                vLost = (currVolume / totalVolume) * 100.0f;
            }
            string text = string.Format("Volume: {0:0.00} %", vLost);
            GUI.Label(rect, text, style);
        }

        if (true)
        {
            float fps = 1.0f / Time.deltaTime;
            float ms = Time.deltaTime * 1000.0f;
            string text = string.Format("{0:N1} FPS ({1:N1}ms)", fps, ms);

            int w = Screen.width, h = Screen.height;
            GUIStyle style = new GUIStyle();
            rectPos = new Rect(0 + xOffset, yOffset, w, h * 2 / 100);
            Rect rect = rectPos;
            style.fontSize = 50;
            style.normal.textColor = color;

            GUI.Label(rect, text, style);
        }
    }
    private void OnDestroy()
    {
        if (this.enabled)
        {
            vertsBuff.Dispose();
            triBuffer.Dispose();
            triBuffer2.Dispose();

            directionCounterBuffer.Dispose();
            directionIntBuffer.Dispose();

            triangleBuffer.Dispose();
            triangleIndicesBuffer.Dispose();

            positionsBuffer.Dispose();
            velocitiesBuffer.Dispose();
            projectedPositionsBuffer.Dispose();
            deltaPositionsBuffer.Dispose();
            deltaPositionsIntBuffer.Dispose();
            distanceConstraintsBuffer.Dispose();
            bendingConstraintsBuffer.Dispose();
            tetVolConstraintsBuffer.Dispose();
            objVolumeBuffer.Dispose();
            deltaCounterBuffer.Dispose();
        }
    }
}
