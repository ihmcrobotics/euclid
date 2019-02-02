package us.ihmc.euclid.shape.convexPolytope.tools;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class EuclidPolytopeFactories
{
   public static ConvexPolytope3D constructUnitCube()
   {
      ConvexPolytope3D polytope = new ConvexPolytope3D();
      polytope.addVertex(new Vertex3D(0.0, 0.0, 0.0));
      polytope.addVertex(new Vertex3D(1.0, 0.0, 0.0));
      polytope.addVertex(new Vertex3D(0.0, 1.0, 0.0));
      polytope.addVertex(new Vertex3D(1.0, 1.0, 0.0));
      polytope.addVertex(new Vertex3D(0.0, 0.0, 1.0));
      polytope.addVertex(new Vertex3D(1.0, 0.0, 1.0));
      polytope.addVertex(new Vertex3D(0.0, 1.0, 1.0));
      polytope.addVertex(new Vertex3D(1.0, 1.0, 1.0));
      return polytope;
   }

   public static ConvexPolytope3D constructExtendedBox(Point3DReadOnly center, Orientation3DReadOnly orientation, double edgeLengthX, double edgeLengthY,
                                                       double edgeLengthZ)
   {
      ConvexPolytope3D polytope = new ConvexPolytope3D();
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotation(orientation);
      polytope.addVertex(center.getX() - edgeLengthX / 2, center.getY() - edgeLengthY / 2, center.getZ() - edgeLengthZ / 2);
      polytope.addVertex(center.getX() + edgeLengthX / 2, center.getY() - edgeLengthY / 2, center.getZ() - edgeLengthZ / 2);
      polytope.addVertex(center.getX() + edgeLengthX / 2, center.getY() + edgeLengthY / 2, center.getZ() - edgeLengthZ / 2);
      polytope.addVertex(center.getX() - edgeLengthX / 2, center.getY() + edgeLengthY / 2, center.getZ() - edgeLengthZ / 2);
      polytope.addVertex(center.getX() - edgeLengthX / 2, center.getY() - edgeLengthY / 2, center.getZ() + edgeLengthZ / 2);
      polytope.addVertex(center.getX() + edgeLengthX / 2, center.getY() - edgeLengthY / 2, center.getZ() + edgeLengthZ / 2);
      polytope.addVertex(center.getX() + edgeLengthX / 2, center.getY() + edgeLengthY / 2, center.getZ() + edgeLengthZ / 2);
      polytope.addVertex(center.getX() - edgeLengthX / 2, center.getY() + edgeLengthY / 2, center.getZ() + edgeLengthZ / 2);
      polytope.applyTransform(transform);
      return polytope;
   }

   public static ConvexPolytope3D constructSphere(Point3DReadOnly center, double radius, int cubeDivisions)
   {
      ConvexPolytope3D polytope = new ConvexPolytope3D();
      List<Vertex3D> vertices = new ArrayList<>();
      for (int i = 0; i < cubeDivisions; i++)
      {
         for (int j = 0; j < cubeDivisions; j++)
         {
            Vertex3D vertex = new Vertex3D((2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius,
                                           (2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius, -radius);
            vertices.add(vertex);
         }
      }

      for (int i = 1; i < cubeDivisions; i++)
      {
         for (int j = 0; j < cubeDivisions; j++)
         {
            Vertex3D vertex = new Vertex3D(-radius, (2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius,
                                           (2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius);
            vertices.add(vertex);
            vertex = new Vertex3D((2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius, -radius,
                                  (2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius);
            vertices.add(vertex);
            vertex = new Vertex3D((2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius, radius,
                                  (2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius);
            vertices.add(vertex);
            vertex = new Vertex3D(radius, (2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius,
                                  (2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius);
            vertices.add(vertex);
         }
      }

      for (int i = 1; i < cubeDivisions - 1; i++)
      {
         for (int j = 1; j < cubeDivisions - 1; j++)
         {
            Vertex3D vertex = new Vertex3D((2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius,
                                           (2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius, radius);
            vertices.add(vertex);
         }
      }

      for (int i = 0; i < vertices.size(); i++)
      {
         Vertex3D vertex = vertices.get(i);
         double mag = Math.sqrt(vertex.getX() * vertex.getX() + vertex.getY() * vertex.getY() + vertex.getZ() * vertex.getZ());
         vertex.setX(vertex.getX() * radius / mag);
         vertex.setY(vertex.getY() * radius / mag);
         vertex.setZ(vertex.getZ() * radius / mag);
         polytope.addVertex(vertex);
      }
      return polytope;
   }

   public static ConvexPolytope3D constructCylinder(Point3DReadOnly center, double radius, double length, int numberOfDivisionsForCurvedSurface)
   {
      ConvexPolytope3D polytope = new ConvexPolytope3D();
      double vertexAngle = 2 * Math.PI / numberOfDivisionsForCurvedSurface;
      double enclosingRadius = radius / Math.cos(vertexAngle / 2.0);
      for (int i = 0; i < numberOfDivisionsForCurvedSurface; i++)
         polytope.addVertex(new Vertex3D(center.getX() + enclosingRadius * Math.cos(i * vertexAngle),
                                         center.getY() + enclosingRadius * Math.sin(i * vertexAngle), center.getZ() - length / 2.0));
      for (int i = 0; i < numberOfDivisionsForCurvedSurface; i++)
      {
         polytope.addVertex(new Vertex3D(center.getX() + enclosingRadius * Math.cos(i * vertexAngle),
                                         center.getY() + enclosingRadius * Math.sin(i * vertexAngle), center.getZ() + length / 2.0));
      }
      return polytope;
   }
}
