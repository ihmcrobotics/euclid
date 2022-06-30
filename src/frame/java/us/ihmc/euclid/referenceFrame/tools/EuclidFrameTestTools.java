package us.ihmc.euclid.referenceFrame.tools;

import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

/**
 * This class provides the tools to perform a variety of assertions on frame geometry types.
 *
 * @author Sylvain Bertrand
 */
public class EuclidFrameTestTools
{
   private static final String DEFAULT_FORMAT = EuclidCoreTestTools.DEFAULT_FORMAT;

   private EuclidFrameTestTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Asserts on a per component basis that the two geometries are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected geometry. Not modified.
    * @param actual   the actual geometry. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two arguments are not equal, not expressed in the same reference
    *                        frame, or only one of the arguments is equal to {@code null}.
    */
   public static void assertEquals(EuclidFrameGeometry expected, EuclidFrameGeometry actual, double epsilon)
   {
      assertEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two geometries are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected geometry. Not modified.
    * @param actual        the actual geometry. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two arguments are not equal, not expressed in the same reference
    *                        frame, or only one of the arguments is equal to {@code null}.
    */
   public static void assertEquals(String messagePrefix, EuclidFrameGeometry expected, EuclidFrameGeometry actual, double epsilon)
   {
      assertEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two geometries are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected geometry. Not modified.
    * @param actual        the actual geometry. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two arguments are not equal, not expressed in the same reference
    *                        frame, or only one of the arguments is equal to {@code null}.
    */
   public static void assertEquals(String messagePrefix, EuclidFrameGeometry expected, EuclidFrameGeometry actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!(expected.epsilonEquals(actual, epsilon)))
      {
         EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts the two geometries are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected geometry. Not modified.
    * @param actual   the actual geometry. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two arguments are not equal, not expressed in the same reference
    *                        frame, or only one of the arguments is equal to {@code null}.
    */
   public static void assertGeometricallyEquals(EuclidFrameGeometry expected, EuclidFrameGeometry actual, double epsilon)
   {
      assertGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts the two geometries are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected geometry. Not modified.
    * @param actual        the actual geometry. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two arguments are not equal, not expressed in the same reference
    *                        frame, or only one of the arguments is equal to {@code null}.
    */
   public static void assertGeometricallyEquals(String messagePrefix, EuclidFrameGeometry expected, EuclidFrameGeometry actual, double epsilon)
   {
      assertGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts the two geometries are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected geometry. Not modified.
    * @param actual        the actual geometry. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two arguments are not equal, not expressed in the same reference
    *                        frame, or only one of the arguments is equal to {@code null}.
    */
   public static void assertGeometricallyEquals(String messagePrefix, EuclidFrameGeometry expected, EuclidFrameGeometry actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!(expected.geometricallyEquals(actual, epsilon)))
      {
         EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }
}
