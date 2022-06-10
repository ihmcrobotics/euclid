package us.ihmc.euclid.referenceFrame.tools;

import us.ihmc.euclid.interfaces.EuclidGeometry;
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
    * Asserts on a per component basis that the two EuclidGeometries are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected EuclidGeomoetry. Not modified.
    * @param actual   the actual EuclidGeomoetry. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two EuclidGeomoetries are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertFrameEquals(EuclidGeometry expected, EuclidGeometry actual, double epsilon)
   {
      assertFrameEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two EuclidGeometries are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected EuclidGeomoetry. Not modified.
    * @param actual        the actual EuclidGeomoetry. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two EuclidGeomoetries are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertFrameEquals(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, double epsilon)
   {
      assertFrameEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two EuclidGeometries are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected EuclidGeomoetry. Not modified.
    * @param actual        the actual EuclidGeomoetry. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two EuclidGeomoetries are not equal. If only one of the arguments
    *                        is equal to {@code null}.
    */
   public static void assertFrameEquals(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!(expected.epsilonEquals(actual, epsilon)))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts the two EuclidGeometries are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected EuclidGeometry. Not modified.
    * @param actual   the actual EuclidGeometry. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two EuclidGeometries do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertFrameGeometricallyEquals(EuclidGeometry expected, EuclidGeometry actual, double epsilon)
   {
      assertFrameGeometricallyEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts the two EuclidGeometries are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected EuclidGeometry. Not modified.
    * @param actual        the actual EuclidGeometry. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two EuclidGeometries do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertFrameGeometricallyEquals(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, double epsilon)
   {
      assertFrameGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts the two EuclidGeometries are geometrically equivalent to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected      the expected EuclidGeometry. Not modified.
    * @param actual        the actual EuclidGeometry. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two EuclidGeometries do not represent the same geometry. If only
    *                        one of the arguments is equal to {@code null}.
    */
   public static void assertFrameGeometricallyEquals(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!(expected.geometricallyEquals(actual, epsilon)))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   private static void throwNotEqualAssertionError(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, String format)
   {
      String expectedAsString = expected.toString(format);
      String actualAsString = actual.toString(format);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, String expectedAsString, String actualAsString)
   {
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, null);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, String expectedAsString, String actualAsString, String differenceAsString)
   {
      String errorMessage = addPrefixToMessage(messagePrefix, "expected:\n" + expectedAsString + "\n but was:\n" + actualAsString);
      if (differenceAsString != null)
         errorMessage += "\nDifference of: " + differenceAsString;

      throw new AssertionError(errorMessage);
   }

   private static String addPrefixToMessage(String prefix, String message)
   {
      if (prefix != null && !prefix.isEmpty())
         return prefix + " " + message;
      else
         return message;
   }
}
