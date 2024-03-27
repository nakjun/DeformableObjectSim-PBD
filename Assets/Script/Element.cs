using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.script
{
    public struct Spring
    {
        public int i1;
        public int i2;
        public float RestLength;
        public Spring(int Index1, int Index2, float restLength)
        {
            i1 = Index1;
            i2 = Index2;
            RestLength = restLength;
        }
    }
    public class TTtriangle
    {
        public int v0;
        public int v1;
        public int v2;

    }
    public struct MTriangle
    {
        public int v0;
        public int v1;
        public int v2;
    }
    public class Triangle
    {
        public int[] vertices;

        public Triangle(int V0, int V1, int V2)
        {
            vertices = new int[3];
            vertices[0] = V0;
            vertices[1] = V1;
            vertices[2] = V2;
        }
    }
    public struct Tetrahedron
    {
        public int i1;
        public int i2;
        public int i3;
        public int i4;
        public float RestVolume;
        public Tetrahedron(int Index1, int Index2, int Index3, int Index4, float restVolume)
        {
            i1 = Index1;
            i2 = Index2;
            i3 = Index3;
            i4 = Index4;
            RestVolume = restVolume;
        }
    }

    public class Edge
    {
        public int startIndex;
        public int endIndex;

        public Edge(int start, int end)
        {
            startIndex = Mathf.Min(start, end);
            endIndex = Mathf.Max(start, end);
        }
    }
    public struct Bending
    {
        public int index0;
        public int index1;
        public int index2;
        public int index3;
        public float restAngle;
    };
    public class EdgeComparer : EqualityComparer<Edge>
    {
        public override int GetHashCode(Edge obj)
        {
            return obj.startIndex * 10000 + obj.endIndex;
        }

        public override bool Equals(Edge x, Edge y)
        {
            return x.startIndex == y.startIndex && x.endIndex == y.endIndex;
        }
    }
    public struct UInt3Struct
    {
        public uint deltaXInt;
        public uint deltaYInt;
        public uint deltaZInt;
    }
    public struct Int3Struct
    {
        public uint deltaXInt;
        public uint deltaYInt;
        public uint deltaZInt;
    }
}
