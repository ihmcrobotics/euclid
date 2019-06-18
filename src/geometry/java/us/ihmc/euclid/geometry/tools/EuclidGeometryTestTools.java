package us.ihmc.euclid.geometry.tools;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools.*;

import us.ihmc.euclid.geometry.LineSegment1D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Triangle3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

/**
 * This class provides the tools to perform a variety of assertions on geometry types.
 * 
 * @author Sylvain Bertrand
 */
public class EuclidGeometryTestTools
{
   private static final String DEFAULT_FORMAT = EuclidCoreTestTools.DEFAULT_FORMAT;

   private EuclidGeometryTestTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Asserts on a per component basis that the two line 2Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected line 2D. Not modified.
    * @param actual   the actual line 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two line 2Ds are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertLine2DEquals(Line2DReadOnly expected, Line2DReadOnly actual, double epsilon)
   {
      assertLine2DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two line 2Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line 2D. Not modified.
    * @param actual        the actual line 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two line 2Ds are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertLine2DEquals(String messagePrefix, Line2DReadOnly expected, Line2DReadOnly actual, double epsilon)
   {
      assertLine2DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two line 2Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line 2D. Not modified.
    * @param actual        the actual line 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two line 2Ds are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertLine2DEquals(String messagePrefix, Line2DReadOnly expected, Line2DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the two line 2Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected line 2D. Not modified.
    * @param actual   the actual line 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two line 2Ds do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertLine2DGeometricallyEquals(Line2DReadOnly expected, Line2DReadOnly actual, double epsilon)
   {
      assertLine2DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two line 2Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line 2D. Not modified.
    * @param actual        the actual line 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two line 2Ds do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertLine2DGeometricallyEquals(String messagePrefix, Line2DReadOnly expected, Line2DReadOnly actual, double epsilon)
   {
      assertLine2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two line 2Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line 2D. Not modified.
    * @param actual        the actual line 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two line 2Ds do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertLine2DGeometricallyEquals(String messagePrefix, Line2DReadOnly expected, Line2DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two line 3Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected line 3D. Not modified.
    * @param actual   the actual line 3D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two line 3Ds are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertLine3DEquals(Line3DReadOnly expected, Line3DReadOnly actual, double epsilon)
   {
      assertLine3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two line 3Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line 3D. Not modified.
    * @param actual        the actual line 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two line 3Ds are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertLine3DEquals(String messagePrefix, Line3DReadOnly expected, Line3DReadOnly actual, double epsilon)
   {
      assertLine3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two line 3Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line 3D. Not modified.
    * @param actual        the actual line 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two line 3Ds are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertLine3DEquals(String messagePrefix, Line3DReadOnly expected, Line3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the two line 3Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected line 3D. Not modified.
    * @param actual   the actual line 3D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two line 3Ds do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertLine3DGeometricallyEquals(Line3DReadOnly expected, Line3DReadOnly actual, double epsilon)
   {
      assertLine3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two line 3Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line 3D. Not modified.
    * @param actual        the actual line 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two line 3Ds do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertLine3DGeometricallyEquals(String messagePrefix, Line3DReadOnly expected, Line3DReadOnly actual, double epsilon)
   {
      assertLine3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two line 3Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line 3D. Not modified.
    * @param actual        the actual line 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two line 3Ds do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertLine3DGeometricallyEquals(String messagePrefix, Line3DReadOnly expected, Line3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two line segment 1Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected line segment 1D. Not modified.
    * @param actual   the actual line segment 1D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two line segment 1Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertLineSegment1DEquals(LineSegment1D expected, LineSegment1D actual, double epsilon)
   {
      assertLineSegment1DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two line segment 1Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line segment 1D. Not modified.
    * @param actual        the actual line segment 1D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two line segment 1Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertLineSegment1DEquals(String messagePrefix, LineSegment1D expected, LineSegment1D actual, double epsilon)
   {
      assertLineSegment1DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two line segment 1Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line segment 1D. Not modified.
    * @param actual        the actual line segment 1D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two line segment 1Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertLineSegment1DEquals(String messagePrefix, LineSegment1D expected, LineSegment1D actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the two line segment 1Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected line segment 1D. Not modified.
    * @param actual   the actual line segment 1D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two line segment 1Ds do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertLineSegment1DGeometricallyEquals(LineSegment1D expected, LineSegment1D actual, double epsilon)
   {
      assertLineSegment1DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two line segment 1Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line segment 1D. Not modified.
    * @param actual        the actual line segment 1D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two line segment 1Ds do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertLineSegment1DGeometricallyEquals(String messagePrefix, LineSegment1D expected, LineSegment1D actual, double epsilon)
   {
      assertLineSegment1DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two line segment 1Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line segment 1D. Not modified.
    * @param actual        the actual line segment 1D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two line segment 1Ds do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertLineSegment1DGeometricallyEquals(String messagePrefix, LineSegment1D expected, LineSegment1D actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two line segment 2Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected line segment 2D. Not modified.
    * @param actual   the actual line segment 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two line segment 2Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertLineSegment2DEquals(LineSegment2DReadOnly expected, LineSegment2DReadOnly actual, double epsilon)
   {
      assertLineSegment2DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two line segment 2Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line segment 2D. Not modified.
    * @param actual        the actual line segment 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two line segment 2Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertLineSegment2DEquals(String messagePrefix, LineSegment2DReadOnly expected, LineSegment2DReadOnly actual, double epsilon)
   {
      assertLineSegment2DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two line segment 2Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line segment 2D. Not modified.
    * @param actual        the actual line segment 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two line segment 2Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertLineSegment2DEquals(String messagePrefix, LineSegment2DReadOnly expected, LineSegment2DReadOnly actual, double epsilon,
                                                String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the two line segment 2Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected line segment 2D. Not modified.
    * @param actual   the actual line segment 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two line segment 2Ds do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertLineSegment2DGeometricallyEquals(LineSegment2DReadOnly expected, LineSegment2DReadOnly actual, double epsilon)
   {
      assertLineSegment2DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two line segment 2Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line segment 2D. Not modified.
    * @param actual        the actual line segment 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two line segment 2Ds do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertLineSegment2DGeometricallyEquals(String messagePrefix, LineSegment2DReadOnly expected, LineSegment2DReadOnly actual, double epsilon)
   {
      assertLineSegment2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two line segment 2Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line segment 2D. Not modified.
    * @param actual        the actual line segment 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two line segment 2Ds do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertLineSegment2DGeometricallyEquals(String messagePrefix, LineSegment2DReadOnly expected, LineSegment2DReadOnly actual, double epsilon,
                                                             String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two line segment 3Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected line segment 3D. Not modified.
    * @param actual   the actual line segment 3D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two line segment 3Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertLineSegment3DEquals(LineSegment3DReadOnly expected, LineSegment3DReadOnly actual, double epsilon)
   {
      assertLineSegment3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two line segment 3Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line segment 3D. Not modified.
    * @param actual        the actual line segment 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two line segment 3Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertLineSegment3DEquals(String messagePrefix, LineSegment3DReadOnly expected, LineSegment3DReadOnly actual, double epsilon)
   {
      assertLineSegment3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two line segment 3Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line segment 3D. Not modified.
    * @param actual        the actual line segment 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two line segment 3Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertLineSegment3DEquals(String messagePrefix, LineSegment3DReadOnly expected, LineSegment3DReadOnly actual, double epsilon,
                                                String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the two line segment 3Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected line segment 3D. Not modified.
    * @param actual   the actual line segment 3D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two line segment 3Ds do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertLineSegment3DGeometricallyEquals(LineSegment3DReadOnly expected, LineSegment3DReadOnly actual, double epsilon)
   {
      assertLineSegment3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two line segment 3Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line segment 3D. Not modified.
    * @param actual        the actual line segment 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two line segment 3Ds do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertLineSegment3DGeometricallyEquals(String messagePrefix, LineSegment3DReadOnly expected, LineSegment3DReadOnly actual, double epsilon)
   {
      assertLineSegment3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two line segment 3Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected line segment 3D. Not modified.
    * @param actual        the actual line segment 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two line segment 3Ds do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertLineSegment3DGeometricallyEquals(String messagePrefix, LineSegment3DReadOnly expected, LineSegment3DReadOnly actual, double epsilon,
                                                             String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
   }

   /**
    * Asserts on a per component basis that the two bounding box 2Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected bounding box 2D. Not modified.
    * @param actual   the actual bounding box 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two bounding box 2Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertBoundingBox2DEquals(BoundingBox2DReadOnly expected, BoundingBox2DReadOnly actual, double epsilon)
   {
      assertBoundingBox2DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two bounding box 2Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected bounding box 2D. Not modified.
    * @param actual        the actual bounding box 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two bounding box 2Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertBoundingBox2DEquals(String messagePrefix, BoundingBox2DReadOnly expected, BoundingBox2DReadOnly actual, double epsilon)
   {
      assertBoundingBox2DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two bounding box 2Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected bounding box 2D. Not modified.
    * @param actual        the actual bounding box 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two bounding box 2Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertBoundingBox2DEquals(String messagePrefix, BoundingBox2DReadOnly expected, BoundingBox2DReadOnly actual, double epsilon,
                                                String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two bounding box 2Ds represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected bounding box 2D. Not modified.
    * @param actual   the actual bounding box 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two bounding box 2Ds do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertBoundingBox2DGeometricallyEquals(BoundingBox2DReadOnly expected, BoundingBox2DReadOnly actual, double epsilon)
   {
      assertBoundingBox2DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two bounding box 2Ds represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected bounding box 2D. Not modified.
    * @param actual        the actual bounding box 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two bounding box 2Ds do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertBoundingBox2DGeometricallyEquals(String messagePrefix, BoundingBox2DReadOnly expected, BoundingBox2DReadOnly actual, double epsilon)
   {
      assertBoundingBox2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two bounding box 2Ds represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected bounding box 2D. Not modified.
    * @param actual        the actual bounding box 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two bounding box 2Ds do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertBoundingBox2DGeometricallyEquals(String messagePrefix, BoundingBox2DReadOnly expected, BoundingBox2DReadOnly actual, double epsilon,
                                                             String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two bounding box 3Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected bounding box 3D. Not modified.
    * @param actual   the actual bounding box 3D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two bounding box 3Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertBoundingBox3DEquals(BoundingBox3DReadOnly expected, BoundingBox3DReadOnly actual, double epsilon)
   {
      assertBoundingBox3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two bounding box 3Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected bounding box 3D. Not modified.
    * @param actual        the actual bounding box 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two bounding box 3Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertBoundingBox3DEquals(String messagePrefix, BoundingBox3DReadOnly expected, BoundingBox3DReadOnly actual, double epsilon)
   {
      assertBoundingBox3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two bounding box 3Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected bounding box 3D. Not modified.
    * @param actual        the actual bounding box 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two bounding box 3Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertBoundingBox3DEquals(String messagePrefix, BoundingBox3DReadOnly expected, BoundingBox3DReadOnly actual, double epsilon,
                                                String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two bounding box 3Ds represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected bounding box 3D. Not modified.
    * @param actual   the actual bounding box 3D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two bounding box 3Ds do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertBoundingBox3DGeometricallyEquals(BoundingBox3DReadOnly expected, BoundingBox3DReadOnly actual, double epsilon)
   {
      assertBoundingBox3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two bounding box 3Ds represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected bounding box 3D. Not modified.
    * @param actual        the actual bounding box 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two bounding box 3Ds do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertBoundingBox3DGeometricallyEquals(String messagePrefix, BoundingBox3DReadOnly expected, BoundingBox3DReadOnly actual, double epsilon)
   {
      assertBoundingBox3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two bounding box 3Ds represent the same geometry to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected bounding box 3D. Not modified.
    * @param actual        the actual bounding box 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two bounding box 3Ds do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertBoundingBox3DGeometricallyEquals(String messagePrefix, BoundingBox3DReadOnly expected, BoundingBox3DReadOnly actual, double epsilon,
                                                             String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two orientation 2Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected orientation 2D. Not modified.
    * @param actual   the actual orientation 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two orientation 2Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertOrientation2DEquals(Orientation2DReadOnly expected, Orientation2DReadOnly actual, double epsilon)
   {
      assertOrientation2DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two orientation 2Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected orientation 2D. Not modified.
    * @param actual        the actual orientation 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two orientation 2Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertOrientation2DEquals(String messagePrefix, Orientation2DReadOnly expected, Orientation2DReadOnly actual, double epsilon)
   {
      assertOrientation2DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two orientation 2Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected orientation 2D. Not modified.
    * @param actual        the actual orientation 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two orientation 2Ds are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertOrientation2DEquals(String messagePrefix, Orientation2DReadOnly expected, Orientation2DReadOnly actual, double epsilon,
                                                String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the two orientation 2Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected orientation 2D. Not modified.
    * @param actual   the actual orientation 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two orientation 2Ds do not represent the same geometry. If only one
    *                        of the arguments is equal to {@code null}.
    */
   public static void assertOrientation2DGeometricallyEquals(Orientation2DReadOnly expected, Orientation2DReadOnly actual, double epsilon)
   {
      assertOrientation2DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two orientation 2Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected orientation 2D. Not modified.
    * @param actual        the actual orientation 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two orientation 2Ds do not represent the same geometry. If only one
    *                        of the arguments is equal to {@code null}.
    */
   public static void assertOrientation2DGeometricallyEquals(String messagePrefix, Orientation2DReadOnly expected, Orientation2DReadOnly actual, double epsilon)
   {
      assertOrientation2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two orientation 2Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected orientation 2D. Not modified.
    * @param actual        the actual orientation 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two orientation 2Ds do not represent the same geometry. If only one
    *                        of the arguments is equal to {@code null}.
    */
   public static void assertOrientation2DGeometricallyEquals(String messagePrefix, Orientation2DReadOnly expected, Orientation2DReadOnly actual, double epsilon,
                                                             String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two plane 3Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected plane 3D. Not modified.
    * @param actual   the actual plane 3D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two plane 3Ds are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertPlane3DEquals(Plane3D expected, Plane3D actual, double epsilon)
   {
      assertPlane3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two plane 3Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected plane 3D. Not modified.
    * @param actual        the actual plane 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two plane 3Ds are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertPlane3DEquals(String messagePrefix, Plane3D expected, Plane3D actual, double epsilon)
   {
      assertPlane3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two plane 3Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected plane 3D. Not modified.
    * @param actual        the actual plane 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two plane 3Ds are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertPlane3DEquals(String messagePrefix, Plane3D expected, Plane3D actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the two plane 3Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected plane 3D. Not modified.
    * @param actual   the actual plane 3D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two plane 3Ds do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertPlane3DGeometricallyEquals(Plane3D expected, Plane3D actual, double epsilon)
   {
      assertPlane3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two plane 3Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected plane 3D. Not modified.
    * @param actual        the actual plane 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two plane 3Ds do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertPlane3DGeometricallyEquals(String messagePrefix, Plane3D expected, Plane3D actual, double epsilon)
   {
      assertPlane3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two plane 3Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected plane 3D. Not modified.
    * @param actual        the actual plane 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two plane 3Ds do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertPlane3DGeometricallyEquals(String messagePrefix, Plane3D expected, Plane3D actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two pose 2Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected pose 2D. Not modified.
    * @param actual   the actual pose 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two pose 2Ds are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertPose2DEquals(Pose2DReadOnly expected, Pose2DReadOnly actual, double epsilon)
   {
      assertPose2DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two pose 2Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected pose 2D. Not modified.
    * @param actual        the actual pose 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two pose 2Ds are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertPose2DEquals(String messagePrefix, Pose2DReadOnly expected, Pose2DReadOnly actual, double epsilon)
   {
      assertPose2DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two pose 2Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected pose 2D. Not modified.
    * @param actual        the actual pose 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two pose 2Ds are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertPose2DEquals(String messagePrefix, Pose2DReadOnly expected, Pose2DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the two pose 2Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected pose 2D. Not modified.
    * @param actual   the actual pose 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two pose 2Ds do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertPose2DGeometricallyEquals(Pose2DReadOnly expected, Pose2DReadOnly actual, double epsilon)
   {
      assertPose2DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two pose 2Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected pose 2D. Not modified.
    * @param actual        the actual pose 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two pose 2Ds do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertPose2DGeometricallyEquals(String messagePrefix, Pose2DReadOnly expected, Pose2DReadOnly actual, double epsilon)
   {
      assertPose2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two pose 2Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected pose 2D. Not modified.
    * @param actual        the actual pose 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two pose 2Ds do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertPose2DGeometricallyEquals(String messagePrefix, Pose2DReadOnly expected, Pose2DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two pose 3D are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected pose 3D. Not modified.
    * @param actual   the actual pose 3D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two pose 3Ds are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertPose3DEquals(Pose3DReadOnly expected, Pose3DReadOnly actual, double epsilon)
   {
      assertPose3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two pose 3Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected pose 3D. Not modified.
    * @param actual        the actual pose 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two pose 3Ds are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertPose3DEquals(String messagePrefix, Pose3DReadOnly expected, Pose3DReadOnly actual, double epsilon)
   {
      assertPose3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two pose 3Ds are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected pose 3D. Not modified.
    * @param actual        the actual pose 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two pose 3Ds are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertPose3DEquals(String messagePrefix, Pose3DReadOnly expected, Pose3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the two pose 3Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected pose 3D. Not modified.
    * @param actual   the actual pose 3D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two pose 3Ds do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertPose3DGeometricallyEquals(Pose3DReadOnly expected, Pose3DReadOnly actual, double epsilon)
   {
      assertPose3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two pose 3Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected pose 3D. Not modified.
    * @param actual        the actual pose 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two pose 3Ds do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertPose3DGeometricallyEquals(String messagePrefix, Pose3DReadOnly expected, Pose3DReadOnly actual, double epsilon)
   {
      assertPose3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two pose 3Ds represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected pose 3D. Not modified.
    * @param actual        the actual pose 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two pose 3Ds do not represent the same geometry. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertPose3DGeometricallyEquals(String messagePrefix, Pose3DReadOnly expected, Pose3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two convex polygon 2D are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected convex polygon 2D. Not modified.
    * @param actual   the actual convex polygon 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two convex polygon 2D are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertConvexPolygon2DEquals(ConvexPolygon2DReadOnly expected, ConvexPolygon2DReadOnly actual, double epsilon)
   {
      assertConvexPolygon2DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two convex polygon 2D are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected convex polygon 2D. Not modified.
    * @param actual        the actual convex polygon 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two convex polygon 2D are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertConvexPolygon2DEquals(String messagePrefix, ConvexPolygon2DReadOnly expected, ConvexPolygon2DReadOnly actual, double epsilon)
   {
      assertConvexPolygon2DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two convex polygon 2D are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected convex polygon 2D. Not modified.
    * @param actual        the actual convex polygon 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two convex polygon 2D are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertConvexPolygon2DEquals(String messagePrefix, ConvexPolygon2DReadOnly expected, ConvexPolygon2DReadOnly actual, double epsilon,
                                                  String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the two convex polygon 2D represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected convex polygon 2D. Not modified.
    * @param actual   the actual convex polygon 2D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two convex polygon 2D do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertConvexPolygon2DGeometricallyEquals(ConvexPolygon2DReadOnly expected, ConvexPolygon2DReadOnly actual, double epsilon)
   {
      assertConvexPolygon2DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two convex polygon 2D represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected convex polygon 2D. Not modified.
    * @param actual        the actual convex polygon 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two convex polygon 2D do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertConvexPolygon2DGeometricallyEquals(String messagePrefix, ConvexPolygon2DReadOnly expected, ConvexPolygon2DReadOnly actual,
                                                               double epsilon)
   {
      assertConvexPolygon2DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two convex polygon 2D represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected convex polygon 2D. Not modified.
    * @param actual        the actual convex polygon 2D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two convex polygon 2D do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertConvexPolygon2DGeometricallyEquals(String messagePrefix, ConvexPolygon2DReadOnly expected, ConvexPolygon2DReadOnly actual,
                                                               double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two triangle 3D are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected triangle 3D. Not modified.
    * @param actual   the actual triangle 3D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two triangle 3D are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertTriangle3DEquals(Triangle3DReadOnly expected, Triangle3DReadOnly actual, double epsilon)
   {
      assertTriangle3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two triangle 3D are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected triangle 3D. Not modified.
    * @param actual        the actual triangle 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two triangle 3D are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertTriangle3DEquals(String messagePrefix, Triangle3DReadOnly expected, Triangle3DReadOnly actual, double epsilon)
   {
      assertTriangle3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two triangle 3D are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected triangle 3D. Not modified.
    * @param actual        the actual triangle 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two triangle 3D are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertTriangle3DEquals(String messagePrefix, Triangle3DReadOnly expected, Triangle3DReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that the two triangle 3D represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected triangle 3D. Not modified.
    * @param actual   the actual triangle 3D. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two triangle 3D do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertTriangle3DGeometricallyEquals(Triangle3DReadOnly expected, Triangle3DReadOnly actual, double epsilon)
   {
      assertTriangle3DGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts that the two triangle 3D represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected triangle 3D. Not modified.
    * @param actual        the actual triangle 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two triangle 3D do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertTriangle3DGeometricallyEquals(String messagePrefix, Triangle3DReadOnly expected, Triangle3DReadOnly actual, double epsilon)
   {
      assertTriangle3DGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts that the two triangle 3D represent the same geometry to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected triangle 3D. Not modified.
    * @param actual        the actual triangle 3D. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two triangle 3D do not represent the same geometry. If only one of
    *                        the arguments is equal to {@code null}.
    */
   public static void assertTriangle3DGeometricallyEquals(String messagePrefix, Triangle3DReadOnly expected, Triangle3DReadOnly actual, double epsilon,
                                                          String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.geometricallyEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Line2DReadOnly expected, Line2DReadOnly actual, String format)
   {
      String expectedAsString = getLine2DString(format, expected);
      String actualAsString = getLine2DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Line3DReadOnly expected, Line3DReadOnly actual, String format)
   {
      String expectedAsString = getLine3DString(format, expected);
      String actualAsString = getLine3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, LineSegment1D expected, LineSegment1D actual, String format)
   {
      String expectedAsString = getLineSegment1DString(format, expected);
      String actualAsString = getLineSegment1DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, LineSegment2DReadOnly expected, LineSegment2DReadOnly actual, String format)
   {
      String expectedAsString = getLineSegment2DString(format, expected);
      String actualAsString = getLineSegment2DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, LineSegment3DReadOnly expected, LineSegment3DReadOnly actual, String format)
   {
      String expectedAsString = getLineSegment3DString(format, expected);
      String actualAsString = getLineSegment3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, BoundingBox2DReadOnly expected, BoundingBox2DReadOnly actual, String format)
   {
      String expectedAsString = getBoundingBox2DString(format, expected);
      String actualAsString = getBoundingBox2DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, BoundingBox3DReadOnly expected, BoundingBox3DReadOnly actual, String format)
   {
      String expectedAsString = getBoundingBox3DString(format, expected);
      String actualAsString = getBoundingBox3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Orientation2DReadOnly expected, Orientation2DReadOnly actual, String format)
   {
      String expectedAsString = getOrientation2DString(format, expected);
      String actualAsString = getOrientation2DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Plane3D expected, Plane3D actual, String format)
   {
      String expectedAsString = getPlane3DString(format, expected);
      String actualAsString = getPlane3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Pose2DReadOnly expected, Pose2DReadOnly actual, String format)
   {
      String expectedAsString = getPose2DString(format, expected);
      String actualAsString = getPose2DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Pose3DReadOnly expected, Pose3DReadOnly actual, String format)
   {
      String expectedAsString = getPose3DString(format, expected);
      String actualAsString = getPose3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, ConvexPolygon2DReadOnly expected, ConvexPolygon2DReadOnly actual, String format)
   {
      String expectedAsString = getConvexPolygon2DString(format, expected);
      String actualAsString = getConvexPolygon2DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, Triangle3DReadOnly expected, Triangle3DReadOnly actual, String format)
   {
      String expectedAsString = getTriangle3DString(format, expected);
      String actualAsString = getTriangle3DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }
}
