package us.ihmc.euclid.geometry.tools;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

/**
 * This class provides the tools to perform a variety of assertions on geometry types.
 *
 * @author Sylvain Bertrand
 */
public class EuclidGeometryTestTools
{
   public static final String DEFAULT_FORMAT = EuclidCoreTestTools.DEFAULT_FORMAT;

   private EuclidGeometryTestTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
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
    * @deprecated Use
    *             {@link EuclidCoreTestTools#assertOrientation2DEquals(Orientation2DReadOnly,Orientation2DReadOnly,double)}
    *             instead
    */
   @Deprecated
   public static void assertOrientation2DEquals(Orientation2DReadOnly expected, Orientation2DReadOnly actual, double epsilon)
   {
      EuclidCoreTestTools.assertEquals(expected, actual, epsilon);
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
    * @deprecated Use
    *             {@link EuclidCoreTestTools#assertOrientation2DEquals(String,Orientation2DReadOnly,Orientation2DReadOnly,double)}
    *             instead
    */
   @Deprecated
   public static void assertOrientation2DEquals(String messagePrefix, Orientation2DReadOnly expected, Orientation2DReadOnly actual, double epsilon)
   {
      EuclidCoreTestTools.assertEquals(messagePrefix, expected, actual, epsilon, EuclidCoreTestTools.DEFAULT_FORMAT);
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
    * @deprecated Use
    *             {@link EuclidCoreTestTools#assertOrientation2DEquals(String,Orientation2DReadOnly,Orientation2DReadOnly,double,String)}
    *             instead
    */
   @Deprecated
   public static void assertOrientation2DEquals(String messagePrefix,
                                                Orientation2DReadOnly expected,
                                                Orientation2DReadOnly actual,
                                                double epsilon,
                                                String format)
   {
      EuclidCoreTestTools.assertEquals(messagePrefix, expected, actual, epsilon, format);
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
    * @deprecated Use
    *             {@link EuclidCoreTestTools#assertOrientation2DGeometricallyEquals(Orientation2DReadOnly,Orientation2DReadOnly,double)}
    *             instead
    */
   @Deprecated
   public static void assertOrientation2DGeometricallyEquals(Orientation2DReadOnly expected, Orientation2DReadOnly actual, double epsilon)
   {
      EuclidCoreTestTools.assertGeometricallyEquals(expected, actual, epsilon);
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
    * @deprecated Use
    *             {@link EuclidCoreTestTools#assertOrientation2DGeometricallyEquals(String,Orientation2DReadOnly,Orientation2DReadOnly,double)}
    *             instead
    */
   @Deprecated
   public static void assertOrientation2DGeometricallyEquals(String messagePrefix, Orientation2DReadOnly expected, Orientation2DReadOnly actual, double epsilon)
   {
      EuclidCoreTestTools.assertGeometricallyEquals(messagePrefix, expected, actual, epsilon, EuclidCoreTestTools.DEFAULT_FORMAT);
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
    * @deprecated Use
    *             {@link EuclidCoreTestTools#assertOrientation2DGeometricallyEquals(String,Orientation2DReadOnly,Orientation2DReadOnly,double,String)}
    *             instead
    */
   @Deprecated
   public static void assertOrientation2DGeometricallyEquals(String messagePrefix,
                                                             Orientation2DReadOnly expected,
                                                             Orientation2DReadOnly actual,
                                                             double epsilon,
                                                             String format)
   {
      EuclidCoreTestTools.assertGeometricallyEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Convenience method for prepending an optional prefix to a message.
    * <p>
    * In the case the given {@code prefix} is {@code null} or empty, the original {@code message} is
    * returned.
    * </p>
    *
    * @param prefix  the {@code String} to prepend to the message.
    * @param message the original message.
    * @return the message with the prefix.
    */
   public static String addPrefixToMessage(String prefix, String message)
   {
      if (prefix != null && !prefix.isEmpty())
         return prefix + " " + message;
      else
         return message;
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
   public static void assertEquals(EuclidGeometry expected, EuclidGeometry actual, double epsilon)
   {
      assertEquals(null, expected, actual, epsilon);
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
   public static void assertEquals(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, double epsilon)
   {
      assertEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertEquals(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, double epsilon, String format)
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
   public static void assertGeometricallyEquals(EuclidGeometry expected, EuclidGeometry actual, double epsilon)
   {
      assertGeometricallyEquals(null, expected, actual, epsilon);
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
   public static void assertGeometricallyEquals(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, double epsilon)
   {
      assertGeometricallyEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
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
   public static void assertGeometricallyEquals(String messagePrefix, EuclidGeometry expected, EuclidGeometry actual, double epsilon, String format)
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

   /**
    * Throws a new {@code AssertionError} as follows:
    *
    * <pre>
    * messagePrefix expected:
    * expectedAsString
    * but was:
    * actualAsString
    * </pre>
    *
    * @param messagePrefix    prefix to add to the error message.
    * @param expectedAsString the result that was expected in a {@code String} form.
    * @param actualAsString   the result that was obtained in a {@code String} form.
    */
   public static void throwNotEqualAssertionError(String messagePrefix, String expectedAsString, String actualAsString)
   {
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, null);
   }

   /**
    * Throws a new {@code AssertionError} as follows:
    *
    * <pre>
    * messagePrefix expected:
    * expectedAsString
    * but was:
    * actualAsString
    * Difference of: differenceAsString
    * </pre>
    *
    * @param messagePrefix      prefix to add to the error message.
    * @param expectedAsString   the result that was expected in a {@code String} form.
    * @param differenceAsString a short comprehensible summary of the difference between the expected
    *                           and obtained results.
    * @param actualAsString     the result that was obtained in a {@code String} form.
    */
   public static void throwNotEqualAssertionError(String messagePrefix, String expectedAsString, String actualAsString, String differenceAsString)
   {
      String errorMessage = addPrefixToMessage(messagePrefix, "expected:\n" + expectedAsString + "\n but was:\n" + actualAsString);
      if (differenceAsString != null)
         errorMessage += "\n" + differenceAsString;

      throw new AssertionError(errorMessage);
   }
}
