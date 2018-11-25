package us.ihmc.euclid.shape.convexPolytope.tools;

import static us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeIOTools.*;

import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

public class EuclidPolytopeTestTools
{
   private static final String DEFAULT_FORMAT = EuclidCoreTestTools.DEFAULT_FORMAT;

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
