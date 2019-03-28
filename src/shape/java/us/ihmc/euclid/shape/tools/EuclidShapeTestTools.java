package us.ihmc.euclid.shape.tools;

import static us.ihmc.euclid.shape.tools.EuclidShapeIOTools.*;

import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import us.ihmc.euclid.shape.collision.Shape3DCollisionTestResult;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidShapeTestTools
{
   private static final String DEFAULT_FORMAT = EuclidCoreTestTools.DEFAULT_FORMAT;

   public static void assertBox3DEquals(Box3DReadOnly expected, Box3DReadOnly actual, double epsilon)
   {
      assertBox3DEquals(null, expected, actual, epsilon);
   }

   public static void assertBox3DEquals(String messagePrefix, Box3DReadOnly expected, Box3DReadOnly actual, double epsilon)
   {
      assertBox3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertBox3DEquals(String messagePrefix, Box3DReadOnly expected, Box3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertBox3DGeometricallyEquals(Box3DReadOnly expected, Box3DReadOnly actual, double epsilon)
   {
      assertBox3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertBox3DGeometricallyEquals(String messagePrefix, Box3DReadOnly expected, Box3DReadOnly actual, double epsilon)
   {
      assertBox3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertBox3DGeometricallyEquals(String messagePrefix, Box3DReadOnly expected, Box3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertCapsule3DEquals(Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon)
   {
      assertCapsule3DEquals(null, expected, actual, epsilon);
   }

   public static void assertCapsule3DEquals(String messagePrefix, Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon)
   {
      assertCapsule3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertCapsule3DEquals(String messagePrefix, Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertCapsule3DGeometricallyEquals(Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon)
   {
      assertCapsule3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertCapsule3DGeometricallyEquals(String messagePrefix, Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon)
   {
      assertCapsule3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertCapsule3DGeometricallyEquals(String messagePrefix, Capsule3DReadOnly expected, Capsule3DReadOnly actual, double epsilon,
                                                         String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertCylinder3DEquals(Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon)
   {
      assertCylinder3DEquals(null, expected, actual, epsilon);
   }

   public static void assertCylinder3DEquals(String messagePrefix, Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon)
   {
      assertCylinder3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertCylinder3DEquals(String messagePrefix, Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertCylinder3DGeometricallyEquals(Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon)
   {
      assertCylinder3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertCylinder3DGeometricallyEquals(String messagePrefix, Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon)
   {
      assertCylinder3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertCylinder3DGeometricallyEquals(String messagePrefix, Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, double epsilon,
                                                          String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertEllipsoid3DEquals(Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon)
   {
      assertEllipsoid3DEquals(null, expected, actual, epsilon);
   }

   public static void assertEllipsoid3DEquals(String messagePrefix, Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon)
   {
      assertEllipsoid3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertEllipsoid3DEquals(String messagePrefix, Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertEllipsoid3DGeometricallyEquals(Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon)
   {
      assertEllipsoid3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertEllipsoid3DGeometricallyEquals(String messagePrefix, Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon)
   {
      assertEllipsoid3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertEllipsoid3DGeometricallyEquals(String messagePrefix, Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, double epsilon,
                                                           String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertPointShape3DEquals(PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon)
   {
      assertPointShape3DEquals(null, expected, actual, epsilon);
   }

   public static void assertPointShape3DEquals(String messagePrefix, PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon)
   {
      assertPointShape3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertPointShape3DEquals(String messagePrefix, PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertPointShape3DGeometricallyEquals(PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon)
   {
      assertPointShape3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertPointShape3DGeometricallyEquals(String messagePrefix, PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon)
   {
      assertPointShape3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertPointShape3DGeometricallyEquals(String messagePrefix, PointShape3DReadOnly expected, PointShape3DReadOnly actual, double epsilon,
                                                            String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertRamp3DEquals(Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon)
   {
      assertRamp3DEquals(null, expected, actual, epsilon);
   }

   public static void assertRamp3DEquals(String messagePrefix, Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon)
   {
      assertRamp3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertRamp3DEquals(String messagePrefix, Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertRamp3DGeometricallyEquals(Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon)
   {
      assertRamp3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertRamp3DGeometricallyEquals(String messagePrefix, Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon)
   {
      assertRamp3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertRamp3DGeometricallyEquals(String messagePrefix, Ramp3DReadOnly expected, Ramp3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertSphere3DEquals(Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon)
   {
      assertSphere3DEquals(null, expected, actual, epsilon);
   }

   public static void assertSphere3DEquals(String messagePrefix, Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon)
   {
      assertSphere3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertSphere3DEquals(String messagePrefix, Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertSphere3DGeometricallyEquals(Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon)
   {
      assertSphere3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertSphere3DGeometricallyEquals(String messagePrefix, Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon)
   {
      assertSphere3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertSphere3DGeometricallyEquals(String messagePrefix, Sphere3DReadOnly expected, Sphere3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertTorus3DEquals(Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon)
   {
      assertTorus3DEquals(null, expected, actual, epsilon);
   }

   public static void assertTorus3DEquals(String messagePrefix, Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon)
   {
      assertTorus3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertTorus3DEquals(String messagePrefix, Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertTorus3DGeometricallyEquals(Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon)
   {
      assertTorus3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertTorus3DGeometricallyEquals(String messagePrefix, Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon)
   {
      assertTorus3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertTorus3DGeometricallyEquals(String messagePrefix, Torus3DReadOnly expected, Torus3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertCollisionTestResultEquals(Shape3DCollisionTestResult expected, Shape3DCollisionTestResult actual, double epsilon)
   {
      assertCollisionTestResultEquals(null, expected, actual, epsilon);
   }

   public static void assertCollisionTestResultEquals(String messagePrefix, Shape3DCollisionTestResult expected, Shape3DCollisionTestResult actual,
                                                      double epsilon)
   {
      assertCollisionTestResultEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertCollisionTestResultEquals(String messagePrefix, Shape3DCollisionTestResult expected, Shape3DCollisionTestResult actual,
                                                      double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         if (expected.areShapesColliding() != actual.areShapesColliding())
         {
            throwNotEqualAssertionError(messagePrefix, expected, actual, format);
         }
         else
         {
            Vector3D differenceNormalOnA = new Vector3D();
            differenceNormalOnA.sub(expected.getNormalOnA(), actual.getNormalOnA());
            Vector3D differenceNormalOnB = new Vector3D();
            differenceNormalOnB.sub(expected.getNormalOnB(), actual.getNormalOnB());

            String difference = "[";
            difference += "distance: " + Math.abs(expected.getDistance() - actual.getDistance());
            difference += ", pointOnA: " + expected.getPointOnA().distance(actual.getPointOnA()) + ", normalOnA: " + differenceNormalOnA.length();
            difference += ", pointOnB: " + expected.getPointOnB().distance(actual.getPointOnB()) + ", normalOnB: " + differenceNormalOnB.length();
            difference += "]";
            throwNotEqualAssertionError(messagePrefix, expected, actual, format, difference);
         }
      }
   }

   public static void assertVertex3DEquals(Vertex3DReadOnly expected, Vertex3DReadOnly actual, double epsilon)
   {
      assertVertex3DEquals(null, expected, actual, epsilon);
   }

   public static void assertVertex3DEquals(String messagePrefix, Vertex3DReadOnly expected, Vertex3DReadOnly actual, double epsilon)
   {
      assertVertex3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertVertex3DEquals(String messagePrefix, Vertex3DReadOnly expected, Vertex3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertVertex3DGeometricallyEquals(Vertex3DReadOnly expected, Vertex3DReadOnly actual, double epsilon)
   {
      assertVertex3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertVertex3DGeometricallyEquals(String messagePrefix, Vertex3DReadOnly expected, Vertex3DReadOnly actual, double epsilon)
   {
      assertVertex3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertVertex3DGeometricallyEquals(String messagePrefix, Vertex3DReadOnly expected, Vertex3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertHalfEdge3DEquals(HalfEdge3DReadOnly expected, HalfEdge3DReadOnly actual, double epsilon)
   {
      assertHalfEdge3DEquals(null, expected, actual, epsilon);
   }

   public static void assertHalfEdge3DEquals(String messagePrefix, HalfEdge3DReadOnly expected, HalfEdge3DReadOnly actual, double epsilon)
   {
      assertHalfEdge3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertHalfEdge3DEquals(String messagePrefix, HalfEdge3DReadOnly expected, HalfEdge3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertHalfEdge3DGeometricallyEquals(HalfEdge3DReadOnly expected, HalfEdge3DReadOnly actual, double epsilon)
   {
      assertHalfEdge3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertHalfEdge3DGeometricallyEquals(String messagePrefix, HalfEdge3DReadOnly expected, HalfEdge3DReadOnly actual, double epsilon)
   {
      assertHalfEdge3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertHalfEdge3DGeometricallyEquals(String messagePrefix, HalfEdge3DReadOnly expected, HalfEdge3DReadOnly actual, double epsilon,
                                                          String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertFace3DEquals(Face3DReadOnly expected, Face3DReadOnly actual, double epsilon)
   {
      assertFace3DEquals(null, expected, actual, epsilon);
   }

   public static void assertFace3DEquals(String messagePrefix, Face3DReadOnly expected, Face3DReadOnly actual, double epsilon)
   {
      assertFace3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertFace3DEquals(String messagePrefix, Face3DReadOnly expected, Face3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertFace3DGeometricallyEquals(Face3DReadOnly expected, Face3DReadOnly actual, double epsilon)
   {
      assertFace3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertFace3DGeometricallyEquals(String messagePrefix, Face3DReadOnly expected, Face3DReadOnly actual, double epsilon)
   {
      assertFace3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertFace3DGeometricallyEquals(String messagePrefix, Face3DReadOnly expected, Face3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertConvexPolytope3DEquals(ConvexPolytope3DReadOnly expected, ConvexPolytope3DReadOnly actual, double epsilon)
   {
      assertConvexPolytope3DEquals(null, expected, actual, epsilon);
   }

   public static void assertConvexPolytope3DEquals(String messagePrefix, ConvexPolytope3DReadOnly expected, ConvexPolytope3DReadOnly actual, double epsilon)
   {
      assertConvexPolytope3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertConvexPolytope3DEquals(String messagePrefix, ConvexPolytope3DReadOnly expected, ConvexPolytope3DReadOnly actual, double epsilon,
                                                   String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertConvexPolytope3DGeometricallyEquals(ConvexPolytope3DReadOnly expected, ConvexPolytope3DReadOnly actual, double epsilon)
   {
      assertConvexPolytope3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertConvexPolytope3DGeometricallyEquals(String messagePrefix, ConvexPolytope3DReadOnly expected, ConvexPolytope3DReadOnly actual,
                                                                double epsilon)
   {
      assertConvexPolytope3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertConvexPolytope3DGeometricallyEquals(String messagePrefix, ConvexPolytope3DReadOnly expected, ConvexPolytope3DReadOnly actual,
                                                                double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   public static void assertConvexPolytope3DGeneralIntegrity(ConvexPolytope3DReadOnly convexPolytope3D)
   {
      assertConvexPolytope3DGeneralIntegrity(null, convexPolytope3D);
   }

   public static void assertConvexPolytope3DGeneralIntegrity(String messagePrefix, ConvexPolytope3DReadOnly convexPolytope3D)
   {
      if (convexPolytope3D.getCentroid().containsNaN())
         EuclidCoreTestTools.throwAssertionError(messagePrefix, "The polytope's centroid contains NaN.");

      assertConvexPolytope3DFacesIntegrity(messagePrefix, convexPolytope3D);
      assertConvexPolytope3DHalfEdgesIntegrity(messagePrefix, convexPolytope3D);
      assertConvexPolytope3DVerticesIntegrity(messagePrefix, convexPolytope3D);
   }

   public static void assertConvexPolytope3DFacesIntegrity(String messagePrefix, ConvexPolytope3DReadOnly convexPolytope3D)
   {
      Set<Face3DReadOnly> faceSet = new HashSet<>();

      for (int faceIndex = 0; faceIndex < convexPolytope3D.getNumberOfFaces(); faceIndex++)
      { // We assert the uniqueness of the faces.
         Face3DReadOnly face = convexPolytope3D.getFace(faceIndex);
         if (!faceSet.add(face))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, faceIndex + "th face is a duplicate.");
      }

      Vector3D toOrigin = new Vector3D();
      Vector3D toDestination = new Vector3D();
      Vector3D toCentroid = new Vector3D();
      Vector3D cross = new Vector3D();

      if (convexPolytope3D.getNumberOfFaces() == 1)
      {
         for (int faceIndex = 0; faceIndex < convexPolytope3D.getNumberOfFaces(); faceIndex++)
         {
            Face3DReadOnly face = convexPolytope3D.getFace(faceIndex);
            Point3DReadOnly centroid = face.getCentroid();
            Vector3DReadOnly normal = face.getNormal();
            List<? extends HalfEdge3DReadOnly> edges = face.getEdges();

            for (int edgeIndex = 0; edgeIndex < edges.size(); edgeIndex++)
            {
               HalfEdge3DReadOnly edge = edges.get(edgeIndex);

               if (edge.getFace() != face)
                  EuclidCoreTestTools.throwAssertionError(messagePrefix, faceIndex + "th face: the " + edgeIndex + "th edge does not this face as its face.");
               if (!edges.contains(edge.getNext()))
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's next does not belong to this face.");
               if (!edges.contains(edge.getPrevious()))
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's previous does not belong to this face.");
               if (edges.indexOf(edge.getNext()) != ((edgeIndex + 1) % edges.size()))
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's next is not at the next index in the list.");
            }

            if (edges.size() >= 3)
            {
               for (int edgeIndex = 0; edgeIndex < edges.size(); edgeIndex++)
               {
                  HalfEdge3DReadOnly edge = edges.get(edgeIndex);

                  if (edge.getFace() != face)
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face: the " + edgeIndex + "th edge does not this face as its face.");
                  if (!edges.contains(edge.getNext()))
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face: the " + edgeIndex + "th edge's next does not belong to this face.");
                  if (!edges.contains(edge.getPrevious()))
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face: the " + edgeIndex + "th edge's previous does not belong to this face.");
                  if (edges.indexOf(edge.getNext()) != ((edgeIndex + 1) % edges.size()))
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face: the " + edgeIndex + "th edge's next is not at the next index in the list.");

                  // Verifying the edges are all clockwise ordered.
                  toOrigin.sub(edge.getOrigin(), centroid);
                  toDestination.sub(edge.getDestination(), centroid);
                  cross.cross(toDestination, toOrigin);

                  if (cross.dot(normal) < 0.0)
                     EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                             faceIndex + "th face: the " + edgeIndex + "th edge is orientated counter-clockwise.");
               }
            }
         }
      }
      else
      {
         for (int faceIndex = 0; faceIndex < convexPolytope3D.getNumberOfFaces(); faceIndex++)
         {
            Face3DReadOnly face = convexPolytope3D.getFace(faceIndex);
            Point3DReadOnly centroid = face.getCentroid();
            Vector3DReadOnly normal = face.getNormal();
            List<? extends HalfEdge3DReadOnly> edges = face.getEdges();

            // Verifying the normal is pointing towards the outside of the polytope.
            toCentroid.sub(centroid, convexPolytope3D.getCentroid());

            if (toCentroid.dot(normal) < 0.0)
            {
               if (convexPolytope3D.getVolume() < convexPolytope3D.getConstructionEpsilon())
                  System.out.println("WARNING: "
                        + EuclidCoreTestTools.addPrefixToMessage(messagePrefix,
                                                                 faceIndex + "th face's normal might be pointing towards the inside of the polytope."));
               else
                  EuclidCoreTestTools.throwAssertionError(messagePrefix, faceIndex + "th face's normal is pointing towards the inside of the polytope.");
            }

            for (int edgeIndex = 0; edgeIndex < edges.size(); edgeIndex++)
            {
               HalfEdge3DReadOnly edge = edges.get(edgeIndex);
               Face3DReadOnly neighbor = edge.getFace();

               if (neighbor != face)
                  EuclidCoreTestTools.throwAssertionError(messagePrefix, faceIndex + "th face: the " + edgeIndex + "th edge does not this face as its face.");
               if (!edges.contains(edge.getNext()))
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's next does not belong to this face.");
               if (!edges.contains(edge.getPrevious()))
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's previous does not belong to this face.");
               if (edges.indexOf(edge.getNext()) != ((edgeIndex + 1) % edges.size()))
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          faceIndex + "th face: the " + edgeIndex + "th edge's next is not at the next index in the list.");

               // Verifying the edges are all clockwise ordered.
               toOrigin.sub(edge.getOrigin(), centroid);
               toDestination.sub(edge.getDestination(), centroid);
               cross.cross(toDestination, toOrigin);

               if (cross.dot(normal) < 0.0)
                  EuclidCoreTestTools.throwAssertionError(messagePrefix, faceIndex + "th face: the " + edgeIndex + "th edge is orientated counter-clockwise.");

               if (face.canObserverSeeFace(neighbor.getCentroid()))
                  EuclidCoreTestTools.throwAssertionError(messagePrefix, faceIndex + "th face is concave with respect to the " + edgeIndex + "th neighbor.");
            }
         }
      }
   }

   public static void assertConvexPolytope3DHalfEdgesIntegrity(String messagePrefix, ConvexPolytope3DReadOnly convexPolytope3D)
   {
      Set<HalfEdge3DReadOnly> halfEdgeSet = new HashSet<>();

      for (int halfEdgeIndex = 0; halfEdgeIndex < convexPolytope3D.getNumberOfHalfEdges(); halfEdgeIndex++)
      { // We assert the uniqueness of the half-edges.
         HalfEdge3DReadOnly halfEdge = convexPolytope3D.getHalfEdge(halfEdgeIndex);
         if (!halfEdgeSet.add(halfEdge))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge is a duplicate.");
      }

      for (int halfEdgeIndex = 0; halfEdgeIndex < convexPolytope3D.getNumberOfHalfEdges(); halfEdgeIndex++)
      {
         HalfEdge3DReadOnly halfEdge = convexPolytope3D.getHalfEdge(halfEdgeIndex);

         Vertex3DReadOnly origin = halfEdge.getOrigin();
         Vertex3DReadOnly destination = halfEdge.getDestination();
         HalfEdge3DReadOnly twin = halfEdge.getTwin();
         HalfEdge3DReadOnly next = halfEdge.getNext();
         HalfEdge3DReadOnly previous = halfEdge.getPrevious();
         Face3DReadOnly face = halfEdge.getFace();

         if (convexPolytope3D.getNumberOfFaces() > 1 && twin == null)
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's twin is null.");
         if (next == null)
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's next is null.");
         if (previous == null)
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's previous is null.");
         if (face == null)
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's face is null.");

         if (twin != null && (origin != twin.getDestination() || destination != twin.getOrigin()))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge is inconsistent with its twin.");
         if (origin != previous.getDestination())
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge is not attached to its previous.");
         if (destination != next.getOrigin())
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge is not attached to its next.");
         if (halfEdge.getFace() != previous.getFace())
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge does not share the same face as its previous.");
         if (halfEdge.getFace() != next.getFace())
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge does not share the same face as its next.");
         if (!origin.getAssociatedEdges().contains(halfEdge))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge is not associated to its origin.");
         if (origin != destination && destination.getAssociatedEdges().contains(halfEdge))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge should not be associated to its origin.");
         if (!halfEdge.getFace().getEdges().contains(halfEdge))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's face does not declare it as one of its edges.");

         if (!convexPolytope3D.getFaces().contains(face))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's face is not registered as a polytope face.");
         if (twin != null && !convexPolytope3D.getHalfEdges().contains(twin))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's twin is not registered as a polytope half-edge.");
         if (!convexPolytope3D.getHalfEdges().contains(next))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's next is not registered as a polytope half-edge.");
         if (!convexPolytope3D.getHalfEdges().contains(previous))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's previous is not registered as a polytope half-edge.");
         if (!convexPolytope3D.getVertices().contains(origin))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's origin is not registered as a polytope vertex.");
         if (!convexPolytope3D.getVertices().contains(destination))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, halfEdgeIndex + "th half-edge's destination is not registered as a polytope vertex.");
      }
   }

   public static void assertConvexPolytope3DVerticesIntegrity(String messagePrefix, ConvexPolytope3DReadOnly convexPolytope3D)
   {
      Set<Vertex3DReadOnly> vertexSet = new HashSet<>();

      for (int vertexIndex = 0; vertexIndex < convexPolytope3D.getNumberOfVertices(); vertexIndex++)
      { // We assert the uniqueness of the vertices.
         Vertex3DReadOnly vertex = convexPolytope3D.getVertex(vertexIndex);
         if (!vertexSet.add(vertex))
            EuclidCoreTestTools.throwAssertionError(messagePrefix, vertexIndex + "th vertex is a duplicate.");
      }

      for (int vertexIndex = 0; vertexIndex < convexPolytope3D.getNumberOfVertices(); vertexIndex++)
      {
         Vertex3DReadOnly vertex = convexPolytope3D.getVertex(vertexIndex);
         Collection<? extends HalfEdge3DReadOnly> associatedEdges = vertex.getAssociatedEdges();

         // Used to assert uniqueness of the faces obtained from the associated-edges.
         Set<Face3DReadOnly> connectedFaces = new HashSet<>();

         for (HalfEdge3DReadOnly associatedEdge : associatedEdges)
         {
            if (vertex != associatedEdge.getOrigin())
               EuclidCoreTestTools.throwAssertionError(messagePrefix, vertexIndex + "th vertex is not the origin of an associated edge.");

            if (!connectedFaces.add(associatedEdge.getFace()))
               EuclidCoreTestTools.throwAssertionError(messagePrefix, "The connected faces to " + vertexIndex + "th vertex are not unique.");

            if (convexPolytope3D.getNumberOfFaces() > 1)
            {
               if (associatedEdge.getDestination().getEdgeTo(vertex) == null)
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          vertexIndex + "th vertex has an edge which destination does not have an edge going back to it.");
               if (associatedEdge.getDestination().getEdgeTo(vertex) != associatedEdge.getTwin())
                  EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                          vertexIndex + "th vertex has an edge which destination's return edge is not the twin.");
            }
            if (!associatedEdge.getFace().getVertices().contains(vertex))
               EuclidCoreTestTools.throwAssertionError(messagePrefix, vertexIndex + "th vertex has an edge which face does not delare it.");

            if (!convexPolytope3D.getFaces().contains(associatedEdge.getFace()))
               EuclidCoreTestTools.throwAssertionError(messagePrefix, vertexIndex + "th vertex has an edge's face which is not registered as a polytope face.");
            if (!convexPolytope3D.getHalfEdges().contains(associatedEdge))
               EuclidCoreTestTools.throwAssertionError(messagePrefix, vertexIndex + "th vertex has an edge which is not registered as a polytope edge.");
            if (!convexPolytope3D.getVertices().contains(associatedEdge.getDestination()))
               EuclidCoreTestTools.throwAssertionError(messagePrefix,
                                                       vertexIndex + "th vertex has an edge's destination which is not registered as a polytope vertex.");
         }
      }
   }

   public static void assertCollisionTestResultGeometricallyEquals(Shape3DCollisionTestResult expected, Shape3DCollisionTestResult actual, double epsilon)
   {
      assertCollisionTestResultGeometricallyEquals(null, expected, actual, epsilon);
   }

   public static void assertCollisionTestResultGeometricallyEquals(String messagePrefix, Shape3DCollisionTestResult expected, Shape3DCollisionTestResult actual,
                                                                   double epsilon)
   {
      assertCollisionTestResultGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertCollisionTestResultGeometricallyEquals(String messagePrefix, Shape3DCollisionTestResult expected, Shape3DCollisionTestResult actual,
                                                                   double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Box3DReadOnly expected, Box3DReadOnly actual, String format)
   {
      String expectedAsString = getBox3DString(format, expected);
      String actualAsString = getBox3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Capsule3DReadOnly expected, Capsule3DReadOnly actual, String format)
   {
      String expectedAsString = getCapsule3DString(format, expected);
      String actualAsString = getCapsule3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Cylinder3DReadOnly expected, Cylinder3DReadOnly actual, String format)
   {
      String expectedAsString = getCylinder3DString(format, expected);
      String actualAsString = getCylinder3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Ellipsoid3DReadOnly expected, Ellipsoid3DReadOnly actual, String format)
   {
      String expectedAsString = getEllipsoid3DString(format, expected);
      String actualAsString = getEllipsoid3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, PointShape3DReadOnly expected, PointShape3DReadOnly actual, String format)
   {
      String expectedAsString = getPointShape3DString(format, expected);
      String actualAsString = getPointShape3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Ramp3DReadOnly expected, Ramp3DReadOnly actual, String format)
   {
      String expectedAsString = getRamp3DString(format, expected);
      String actualAsString = getRamp3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Sphere3DReadOnly expected, Sphere3DReadOnly actual, String format)
   {
      String expectedAsString = getSphere3DString(format, expected);
      String actualAsString = getSphere3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Torus3DReadOnly expected, Torus3DReadOnly actual, String format)
   {
      String expectedAsString = getTorus3DString(format, expected);
      String actualAsString = getTorus3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Shape3DCollisionTestResult expected, Shape3DCollisionTestResult actual, String format)
   {
      throwNotEqualAssertionError(messagePrefix, expected, actual, format, null);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Shape3DCollisionTestResult expected, Shape3DCollisionTestResult actual, String format,
                                                   String differenceAsString)
   {
      String expectedAsString = getCollisionTestResultString(format, expected);
      String actualAsString = getCollisionTestResultString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, differenceAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Vertex3DReadOnly expected, Vertex3DReadOnly actual, String format)
   {
      String expectedAsString = getVertex3DString(format, expected);
      String actualAsString = getVertex3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, HalfEdge3DReadOnly expected, HalfEdge3DReadOnly actual, String format)
   {
      String expectedAsString = getHalfEdge3DString(format, expected);
      String actualAsString = getHalfEdge3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Face3DReadOnly expected, Face3DReadOnly actual, String format)
   {
      String expectedAsString = getFace3DString(format, expected);
      String actualAsString = getFace3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, ConvexPolytope3DReadOnly expected, ConvexPolytope3DReadOnly actual, String format)
   {
      String expectedAsString = getConvexPolytope3DString(format, expected);
      String actualAsString = getConvexPolytope3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }
}
