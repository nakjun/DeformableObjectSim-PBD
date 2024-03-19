using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;
using System.Linq;
using Assets.script;

public class LoadTetModel : MonoBehaviour
{
    private static GameObject obj;

    public static List<Vector3> positions = new List<Vector3>();
    public static List<Triangle> triangles = new List<Triangle>(); //for compute
    public static List<Tetrahedron> tetrahedrons = new List<Tetrahedron>();
    public static List<Spring> springs = new List<Spring>();
    public static List<Bending> bendings = new List<Bending>();

    public static List<int> triangleArr = new List<int>();//for render
    public static List<int> springColor = new List<int>();
    public static int totalColor;

    static int nodeCount;
    static int triCount;
    static int tetCount;
    static int springCount;
    static int bendingCount;
    static string fileName = "";
    private static float computeTetraVolume(Vector3 i1, Vector3 i2, Vector3 i3, Vector3 i4)
    {
        float volume = 0.0f;

        volume = 1.0f / 6.0f
            * (i3.x * i2.y * i1.z - i4.x * i2.y * i1.z - i2.x * i3.y * i1.z
            + i4.x * i3.y * i1.z + i2.x * i4.y * i1.z - i3.x * i4.y * i1.z
            - i3.x * i1.y * i2.z + i4.x * i1.y * i2.z + i1.x * i3.y * i2.z
            - i4.x * i3.y * i2.z - i1.x * i4.y * i2.z + i3.x * i4.y * i2.z
            + i2.x * i1.y * i3.z - i4.x * i1.y * i3.z - i1.x * i2.y * i3.z
            + i4.x * i2.y * i3.z + i1.x * i4.y * i3.z - i2.x * i4.y * i3.z
            - i2.x * i1.y * i4.z + i3.x * i1.y * i4.z + i1.x * i2.y * i4.z
            - i3.x * i2.y * i4.z - i1.x * i3.y * i4.z + i2.x * i3.y * i4.z);

        return volume;
    }

    public LoadTetModel(string filename)
    {
        fileName = filename;
    }
    public LoadTetModel(string filename, GameObject gameobj)
    {
        fileName = filename;
        obj = gameobj;
    }
    public static void LoadData(string filename, GameObject gameobj)
    {
        fileName = filename;
        obj = gameobj;
        //print("start load data !");
        loadNodesPosition();
        loadFaces();
        loadTetrahedron();
        loadSpring();
        loadBending();
        //print("finish load data !");
    }

    private static void loadNodesPosition()
    {
        string Nodepath = fileName + ".node";
        string line;
        StreamReader reader1 = new StreamReader(Nodepath);

        Matrix4x4 sMatrix = Matrix4x4.Scale(obj.transform.localScale);
        Matrix4x4 tMatrix = Matrix4x4.Translate(obj.transform.position);
        Quaternion rotation = Quaternion.Euler(obj.transform.eulerAngles.x,
                                               obj.transform.eulerAngles.y, 
                                               obj.transform.eulerAngles.z);        
        Matrix4x4 rMatrix = Matrix4x4.Rotate(rotation);

        using (reader1)
        {
            line = reader1.ReadLine();
            do
            {
                line = reader1.ReadLine(); // first line
                if (line != null)
                {
                    string[] tmpPosPerRow = line.Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                    if (tmpPosPerRow.Length > 0 && tmpPosPerRow.Length < 5) // if data
                    {
                        Vector3 pos = new Vector3(0, 0, 0);
                        pos.x = float.Parse(tmpPosPerRow[1]);
                        pos.y = float.Parse(tmpPosPerRow[2]);
                        pos.z = float.Parse(tmpPosPerRow[3]);

                        pos = rMatrix.MultiplyPoint(pos);
                        pos = sMatrix.MultiplyPoint(pos);
                        pos = tMatrix.MultiplyPoint(pos);

                        positions.Add(pos);
                    }
                }
            }
            while (line != null);
            reader1.Close();
        }
        nodeCount = positions.Count;
        //print(nodeCount);
    }
    private static void loadFaces()
    {
        string Facepath = fileName + ".face";
        StreamReader reader1 = new StreamReader(Facepath);
        string line;
        using (reader1)
        {
            line = reader1.ReadLine();
            do
            {
                line = reader1.ReadLine(); // first line
                if (line != null)
                {
                    string[] tmpPosPerRow = line.Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                    if (tmpPosPerRow.Length > 0 && tmpPosPerRow.Length < 6) // if data
                    {
                        //Vector3 face = new Vector3(0, 0, 0);

                        triangleArr.Add(int.Parse(tmpPosPerRow[1]));
                        triangleArr.Add(int.Parse(tmpPosPerRow[3]));
                        triangleArr.Add(int.Parse(tmpPosPerRow[2]));
                        //1,3,2 for torus

                        //faces.Add(Triangle(tmpPosPerRow[1], tmpPosPerRow[3], tmpPosPerRow[2]))
                        Triangle t;
                        if (fileName == "cow.1" || fileName == "dragon_vrip_res3.1")
                            t = new Triangle(int.Parse(tmpPosPerRow[1]),
                                int.Parse(tmpPosPerRow[2]), int.Parse(tmpPosPerRow[3]));
                        else
                            t = new Triangle(int.Parse(tmpPosPerRow[1]),
                                int.Parse(tmpPosPerRow[3]), int.Parse(tmpPosPerRow[2]));

                        triangles.Add(t);
                    }
                }
            }
            while (line != null);
            reader1.Close();
        }
        triCount = triangles.Count;
    }
    private static void loadTetrahedron()
    {
        string ElePath = fileName + ".ele";
        StreamReader reader1 = new StreamReader(ElePath);
        string line;
        using (reader1)
        {
            line = reader1.ReadLine();
            do
            {
                line = reader1.ReadLine(); // first line
                if (line != null)
                {
                    string[] tmpPosPerRow = line.Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                    if (tmpPosPerRow.Length > 0 && tmpPosPerRow.Length < 6) // if data
                    {


                        int i0 = int.Parse(tmpPosPerRow[1]);
                        int i1 = int.Parse(tmpPosPerRow[2]);
                        int i2 = int.Parse(tmpPosPerRow[3]);
                        int i3 = int.Parse(tmpPosPerRow[4]);
                        float volume = 0;
                        volume = computeTetraVolume(positions[i0], positions[i1],
                            positions[i2], positions[i3]);

                        //if(i0 >= nodeCount || i1 >= nodeCount || i2 >= nodeCount || i3 >= nodeCount)
                        //{
                        //    print("Error :: " + tetrahedrons.Count);
                        //}

                        Tetrahedron tetrahedron = new Tetrahedron(i0, i1, i2, i3, volume);

                        //print(i0 + "," + i1 + "," + i2 + "," + i3);
                        tetrahedrons.Add(tetrahedron);
                    }
                }
            }
            while (line != null);
            reader1.Close();

        }
        tetCount = tetrahedrons.Count;
    }
    private static void loadSpring()
    {
        string springpath = fileName + ".springinterior";
        StreamReader reader1 = new StreamReader(springpath);
        string line;

        using (reader1)
        {
            line = reader1.ReadLine();
            string[] tmpData = line.Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            totalColor = int.Parse(tmpData[1]);

            do
            {
                line = reader1.ReadLine(); // first line
                if (line != null)
                {
                    string[] tmpPosPerRow = line.Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                    //print(tmpPosPerRow.Length);
                    if (tmpPosPerRow.Length > 2) // if data
                    {

                        int i1 = int.Parse(tmpPosPerRow[0]);
                        int i2 = int.Parse(tmpPosPerRow[1]);
                        int color = int.Parse(tmpPosPerRow[2]);
                        Spring s = initSpring(i1, i2);
                        springColor.Add(color);


                        springs.Add(s);
                    }
                    //else if (tmpPosPerRow.Length == 3)
                    //{
                    //    totalColor = int.Parse(tmpPosPerRow[1]);
                    //    print("run");
                    //}
                }
            }
            while (line != null);
            reader1.Close();
        }
        springCount = springs.Count;
    }
    private static void loadBending()
    {
        string Facepath = fileName + ".bending";
        StreamReader reader1 = new StreamReader(Facepath);
        string line;
        using (reader1)
        {
            line = reader1.ReadLine();
            do
            {
                line = reader1.ReadLine(); // first line
                if (line != null)
                {
                    string[] tmpPosPerRow = line.Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                    if (tmpPosPerRow.Length > 1 && tmpPosPerRow.Length < 6) // if data
                    {
                        Vector3 p0 = positions[(int.Parse(tmpPosPerRow[1]))];
                        Vector3 p1 = positions[(int.Parse(tmpPosPerRow[2]))];
                        Vector3 p2 = positions[(int.Parse(tmpPosPerRow[3]))];
                        Vector3 p3 = positions[(int.Parse(tmpPosPerRow[4]))];

                        Bending b = new Bending();

                        Vector3 n1 = (Vector3.Cross(p2 - p0, p3 - p0)).normalized;
                        Vector3 n2 = (Vector3.Cross(p3 - p1, p2 - p1)).normalized;
                        float d = Vector3.Dot(n1, n2);
                        d = Mathf.Clamp(d, -1.0f, 1.0f);

                        b.index0 = (int.Parse(tmpPosPerRow[1]));
                        b.index1 = (int.Parse(tmpPosPerRow[2]));
                        b.index2 = (int.Parse(tmpPosPerRow[3]));
                        b.index3 = (int.Parse(tmpPosPerRow[4]));
                        b.restAngle = Mathf.Acos(d);

                        bendings.Add(b);
                    }
                }
            }
            while (line != null);
            reader1.Close();
        }
        bendingCount = bendings.Count;
    }

    static Spring initSpring(int i1, int i2)
    {
        float rl = Vector3.Distance(positions[i1], positions[i2]);
        Spring spring = new Spring(i1, i2, rl);
        //print(i1 + "," + i2);

        return spring;
    }
    public static void ClearData()
    {
        positions = new List<Vector3>();
        triangles = new List<Triangle>(); //for compute
        tetrahedrons = new List<Tetrahedron>();
        springs = new List<Spring>();
        triangleArr = new List<int>();//for render
        bendings = new List<Bending>();
        springColor = new List<int>();

        nodeCount = 0;
        springCount = 0;
        triCount = 0;
        tetCount = 0;
        bendingCount = 0;
        totalColor = 0;

    }
}
