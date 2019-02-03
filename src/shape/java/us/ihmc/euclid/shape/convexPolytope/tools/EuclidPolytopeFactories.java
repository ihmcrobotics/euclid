package us.ihmc.euclid.shape.convexPolytope.tools;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.tuple3D.Point3D;

public class EuclidPolytopeFactories
{
   public static List<Point3D> newIcosahedronVertices(double radius)
   {
      return newIcoSphereVertices(radius, 0);
   }

   public static List<Point3D> newIcoSphereVertices(double radius, int recursionLevel)
   {
      List<Point3D> vertices = IcoSphereFactory.newIcoSphere(recursionLevel).getVertices();
      if (radius != 1.0)
         vertices.forEach(vertex -> vertex.scale(radius));
      return vertices;
   }

   public static List<Point3D> newConeVertices(double height, double radius, int numberOfDivisions)
   {
      List<Point3D> vertices = new ArrayList<>();
      vertices.add(new Point3D(0.0, 0.0, height));
      for (int i = 0; i < numberOfDivisions; i++)
      {
         double theta = i * 2.0 * Math.PI / numberOfDivisions;
         double x = radius * Math.cos(theta);
         double y = radius * Math.sin(theta);
         vertices.add(new Point3D(x, y, 0.0));
      }
      return vertices;
   }

   public static List<Point3D> newCubeVertices(double edgeLength)
   {
      List<Point3D> vertices = new ArrayList<>();
      double halfL = 0.5 * edgeLength;
      vertices.add(new Point3D(-halfL, -halfL, -halfL));
      vertices.add(new Point3D(halfL, -halfL, -halfL));
      vertices.add(new Point3D(-halfL, halfL, -halfL));
      vertices.add(new Point3D(halfL, halfL, -halfL));
      vertices.add(new Point3D(-halfL, -halfL, halfL));
      vertices.add(new Point3D(halfL, -halfL, halfL));
      vertices.add(new Point3D(-halfL, halfL, halfL));
      vertices.add(new Point3D(halfL, halfL, halfL));
      return vertices;
   }

   public static List<Point3D> newCylinderVertices(double length, double radius, int numberOfDivisions)
   {
      List<Point3D> vertices = new ArrayList<>();

      for (int i = 0; i < numberOfDivisions; i++)
      {
         double theta = i * 2.0 * Math.PI / numberOfDivisions;

         double x = radius * Math.cos(theta);
         double y = radius * Math.sin(theta);
         Point3D top = new Point3D(x, y, 0.5 * length);
         Point3D bottom = new Point3D(x, y, -0.5 * length);

         vertices.add(top);
         vertices.add(bottom);
      }
      return vertices;
   }

   public static List<Point3D> newPyramidVertices(double height, double baseLength, double baseWidth)
   {
      List<Point3D> vertices = new ArrayList<>();
      vertices.add(new Point3D(0.0, 0.0, height));
      vertices.add(new Point3D(0.5 * baseLength, 0.5 * baseWidth, 0.0));
      vertices.add(new Point3D(0.5 * baseLength, -0.5 * baseWidth, 0.0));
      vertices.add(new Point3D(-0.5 * baseLength, 0.5 * baseWidth, 0.0));
      vertices.add(new Point3D(-0.5 * baseLength, -0.5 * baseWidth, 0.0));
      return vertices;
   }

   public static ConvexPolytope3D newIcosahedron(double radius)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(newIcosahedronVertices(radius)));
   }

   public static ConvexPolytope3D newIcoSphere(double radius, int recursionLevel)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(newIcoSphereVertices(radius, recursionLevel)));
   }

   public static ConvexPolytope3D newCone(double height, double radius, int numberOfDivisions)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(newConeVertices(height, radius, numberOfDivisions)));
   }

   public static ConvexPolytope3D newCube(double edgeLength)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(newCubeVertices(edgeLength)));
   }

   public static ConvexPolytope3D newCylinder(double length, double radius, int numberOfDivisions)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(newCylinderVertices(length, radius, numberOfDivisions)));
   }

   public static ConvexPolytope3D newPyramid(double height, double baseLength, double baseWidth)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(newPyramidVertices(height, baseLength, baseWidth)));
   }
}
