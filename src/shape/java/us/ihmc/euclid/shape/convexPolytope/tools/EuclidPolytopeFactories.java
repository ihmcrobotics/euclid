package us.ihmc.euclid.shape.convexPolytope.tools;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class EuclidPolytopeFactories
{
   public static List<Point3D> newIcosahedronVertices(double radius)
   {
      List<Point3D> vertices = IcoSphereFactory.newIcoSphere(0).getVertices();
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

   public static ConvexPolytope3D newCone(double height, double radius, int numberOfDivisions)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(newConeVertices(height, radius, numberOfDivisions)));
   }

   public static ConvexPolytope3D newCylinder(double length, double radius, int numberOfDivisions)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(newCylinderVertices(length, radius, numberOfDivisions)));
   }

   public static ConvexPolytope3D newPyramid(double height, double baseLength, double baseWidth)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(newPyramidVertices(height, baseLength, baseWidth)));
   }

   public static ConvexPolytope3D constructUnitCube()
   {
      ConvexPolytope3D polytope = new ConvexPolytope3D();
      polytope.addVertex(new Point3D(0.0, 0.0, 0.0));
      polytope.addVertex(new Point3D(1.0, 0.0, 0.0));
      polytope.addVertex(new Point3D(0.0, 1.0, 0.0));
      polytope.addVertex(new Point3D(1.0, 1.0, 0.0));
      polytope.addVertex(new Point3D(0.0, 0.0, 1.0));
      polytope.addVertex(new Point3D(1.0, 0.0, 1.0));
      polytope.addVertex(new Point3D(0.0, 1.0, 1.0));
      polytope.addVertex(new Point3D(1.0, 1.0, 1.0));
      return polytope;
   }

   public static ConvexPolytope3D constructExtendedBox(Point3DReadOnly center, Orientation3DReadOnly orientation, double edgeLengthX, double edgeLengthY,
                                                       double edgeLengthZ)
   {
      ConvexPolytope3D polytope = new ConvexPolytope3D();
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotation(orientation);
      polytope.addVertex(new Point3D(center.getX() - edgeLengthX / 2, center.getY() - edgeLengthY / 2, center.getZ() - edgeLengthZ / 2));
      polytope.addVertex(new Point3D(center.getX() + edgeLengthX / 2, center.getY() - edgeLengthY / 2, center.getZ() - edgeLengthZ / 2));
      polytope.addVertex(new Point3D(center.getX() + edgeLengthX / 2, center.getY() + edgeLengthY / 2, center.getZ() - edgeLengthZ / 2));
      polytope.addVertex(new Point3D(center.getX() - edgeLengthX / 2, center.getY() + edgeLengthY / 2, center.getZ() - edgeLengthZ / 2));
      polytope.addVertex(new Point3D(center.getX() - edgeLengthX / 2, center.getY() - edgeLengthY / 2, center.getZ() + edgeLengthZ / 2));
      polytope.addVertex(new Point3D(center.getX() + edgeLengthX / 2, center.getY() - edgeLengthY / 2, center.getZ() + edgeLengthZ / 2));
      polytope.addVertex(new Point3D(center.getX() + edgeLengthX / 2, center.getY() + edgeLengthY / 2, center.getZ() + edgeLengthZ / 2));
      polytope.addVertex(new Point3D(center.getX() - edgeLengthX / 2, center.getY() + edgeLengthY / 2, center.getZ() + edgeLengthZ / 2));
      polytope.applyTransform(transform);
      return polytope;
   }

   public static ConvexPolytope3D constructSphere(Point3DReadOnly center, double radius, int cubeDivisions)
   {
      ConvexPolytope3D polytope = new ConvexPolytope3D();
      List<Point3D> vertices = new ArrayList<>();
      for (int i = 0; i < cubeDivisions; i++)
      {
         for (int j = 0; j < cubeDivisions; j++)
         {
            Point3D vertex = new Point3D((2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius,
                                         (2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius, -radius);
            vertices.add(vertex);
         }
      }

      for (int i = 1; i < cubeDivisions; i++)
      {
         for (int j = 0; j < cubeDivisions; j++)
         {
            Point3D vertex = new Point3D(-radius, (2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius,
                                         (2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius);
            vertices.add(vertex);
            vertex = new Point3D((2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius, -radius,
                                 (2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius);
            vertices.add(vertex);
            vertex = new Point3D((2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius, radius,
                                 (2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius);
            vertices.add(vertex);
            vertex = new Point3D(radius, (2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius,
                                 (2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius);
            vertices.add(vertex);
         }
      }

      for (int i = 1; i < cubeDivisions - 1; i++)
      {
         for (int j = 1; j < cubeDivisions - 1; j++)
         {
            Point3D vertex = new Point3D((2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius,
                                         (2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius, radius);
            vertices.add(vertex);
         }
      }

      for (int i = 0; i < vertices.size(); i++)
      {
         Point3D vertex = vertices.get(i);
         double mag = Math.sqrt(vertex.getX() * vertex.getX() + vertex.getY() * vertex.getY() + vertex.getZ() * vertex.getZ());
         vertex.setX(vertex.getX() * radius / mag);
         vertex.setY(vertex.getY() * radius / mag);
         vertex.setZ(vertex.getZ() * radius / mag);
         polytope.addVertex(vertex);
      }
      return polytope;
   }
}
