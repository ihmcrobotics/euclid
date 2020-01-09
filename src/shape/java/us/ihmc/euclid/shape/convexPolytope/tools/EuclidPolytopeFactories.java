package us.ihmc.euclid.shape.convexPolytope.tools;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * This class provides a set of factories to create primitive shapes as convex polytopes.
 *
 * @author Sylvain Bertrand
 */
public class EuclidPolytopeFactories
{
   private EuclidPolytopeFactories()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Discretizes a cone shape and returns a list of vertices representing it.
    * <p>
    * The cone has its base center at the origin and its axis of revolution aligned to the z-axis.
    * </p>
    *
    * @param height            the height of the cone.
    * @param radius            the radius of the cone base.
    * @param numberOfDivisions the number of divisions for discretizing the cone.
    * @return the cone vertices.
    */
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

   /**
    * Creates the list of vertices of a cube.
    * <p>
    * The cube is axis-aligned and is centered at the origin.
    * </p>
    *
    * @param edgeLength the cube edge length.
    * @return the cube vertices.
    */
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

   /**
    * Discretizes a cylinder shape and returns a list of vertices representing it.
    * <p>
    * The cylinder is centered at the origin and its axis of revolution is aligned to the z-axis.
    * </p>
    *
    * @param length            the length of the cylinder.
    * @param radius            the radius of the cylinder.
    * @param numberOfDivisions the number of divisions for discretizing the cylinder.
    * @return the cylinder vertices.
    */
   public static List<Point3D> newCylinderVertices(double length, double radius, int numberOfDivisions)
   {
      List<Point3D> vertices = new ArrayList<>();
      double halfL = 0.5 * length;

      for (int i = 0; i < numberOfDivisions; i++)
      {
         double theta = i * 2.0 * Math.PI / numberOfDivisions;

         double x = radius * Math.cos(theta);
         double y = radius * Math.sin(theta);
         Point3D top = new Point3D(x, y, halfL);
         Point3D bottom = new Point3D(x, y, -halfL);

         vertices.add(top);
         vertices.add(bottom);
      }
      return vertices;
   }

   /**
    * Creates the list of vertices of a icosahedron.
    * <p>
    * The icosahedron is centered at the origin.
    * </p>
    *
    * @param radius the radius of the circumscribed sphere of the icosahedron.
    * @return the icosahedron vertices.
    */
   public static List<Point3D> newIcosahedronVertices(double radius)
   {
      return newIcoSphereVertices(radius, 0);
   }

   /**
    * Discretizes a sphere using the {@link IcoSphereFactory} and returns a list of vertices
    * representing it.
    *
    * @param radius         the sphere radius.
    * @param recursionLevel the resolution.
    * @return the sphere vertices.
    * @see IcoSphereFactory
    */
   public static List<Point3D> newIcoSphereVertices(double radius, int recursionLevel)
   {
      List<Point3D> vertices = IcoSphereFactory.newIcoSphere(recursionLevel).getVertices();
      if (radius != 1.0)
         vertices.forEach(vertex -> vertex.scale(radius));
      return vertices;
   }

   /**
    * Creates the list of vertices of a pyramid.
    * <p>
    * The pyramid is axis-aligned and its base is centered at the origin.
    * </p>
    *
    * @param height     the height of the pyramid.
    * @param baseLength the length of the pyramid base.
    * @param baseWidth  the width of the pyramid base.
    * @return the pyramid vertices.
    */
   public static List<Point3D> newPyramidVertices(double height, double baseLength, double baseWidth)
   {
      List<Point3D> vertices = new ArrayList<>();
      double halfL = 0.5 * baseLength;
      double halfW = 0.5 * baseWidth;

      vertices.add(new Point3D(0.0, 0.0, height));
      vertices.add(new Point3D(halfL, halfW, 0.0));
      vertices.add(new Point3D(halfL, -halfW, 0.0));
      vertices.add(new Point3D(-halfL, halfW, 0.0));
      vertices.add(new Point3D(-halfL, -halfW, 0.0));
      return vertices;
   }

   /**
    * Creates a new convex polytope from discretizing a cone.
    * <p>
    * The cone has its base center at the origin and its axis of revolution aligned to the z-axis.
    * </p>
    *
    * @param height            the height of the cone.
    * @param radius            the radius of the cone base.
    * @param numberOfDivisions the number of divisions for discretizing the cone.
    * @return the convex polytope representing a cone.
    */
   public static ConvexPolytope3D newCone(double height, double radius, int numberOfDivisions)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(newConeVertices(height, radius, numberOfDivisions)));
   }

   /**
    * Creates a new convex polytope to represent a cube.
    * <p>
    * The cube is axis-aligned and is centered at the origin.
    * </p>
    *
    * @param edgeLength the cube edge length.
    * @return the convex polytope representing a cube.
    */
   public static ConvexPolytope3D newCube(double edgeLength)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(newCubeVertices(edgeLength)));
   }

   /**
    * Creates a new convex polytope from discretizing a cylinder.
    * <p>
    * The cylinder is centered at the origin and its axis of revolution is aligned to the z-axis.
    * </p>
    *
    * @param length            the length of the cylinder.
    * @param radius            the radius of the cylinder.
    * @param numberOfDivisions the number of divisions for discretizing the cylinder.
    * @return the convex polytope representing a cylinder.
    */
   public static ConvexPolytope3D newCylinder(double length, double radius, int numberOfDivisions)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(newCylinderVertices(length, radius, numberOfDivisions)));
   }

   /**
    * Creates a new convex polytope to represent a icosahedron.
    * <p>
    * The icosahedron is centered at the origin.
    * </p>
    *
    * @param radius the radius of the circumscribed sphere of the icosahedron.
    * @return the convex polytope representing a icosahedron.
    */
   public static ConvexPolytope3D newIcosahedron(double radius)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(newIcosahedronVertices(radius)));
   }

   /**
    * Creates a new convex polytope to represent a ico-sphere.
    *
    * @param radius         the sphere radius.
    * @param recursionLevel the resolution.
    * @return the convex polytope representing a ico-sphere.
    * @see IcoSphereFactory
    */
   public static ConvexPolytope3D newIcoSphere(double radius, int recursionLevel)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(newIcoSphereVertices(radius, recursionLevel)));
   }

   /**
    * Creates a new convex polytope to represent a pyramid.
    * <p>
    * The pyramid is axis-aligned and its base is centered at the origin.
    * </p>
    *
    * @param height     the height of the pyramid.
    * @param baseLength the length of the pyramid base.
    * @param baseWidth  the width of the pyramid base.
    * @return the convex polytope representing a pyramid.
    */
   public static ConvexPolytope3D newPyramid(double height, double baseLength, double baseWidth)
   {
      return new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(newPyramidVertices(height, baseLength, baseWidth)));
   }
}
