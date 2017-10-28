package us.ihmc.euclid.referenceFrame.tools;

import static us.ihmc.euclid.tools.EuclidCoreIOTools.getStringFormat;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getTuple2DString;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getTuple3DString;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getTuple4DString;
import static us.ihmc.euclid.tools.EuclidCoreTestTools.assertRotationVectorEquals;
import static us.ihmc.euclid.tools.EuclidCoreTestTools.assertTuple2DEquals;
import static us.ihmc.euclid.tools.EuclidCoreTestTools.assertTuple3DEquals;
import static us.ihmc.euclid.tools.EuclidCoreTestTools.assertTuple4DEquals;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;

public class EuclidFrameTestTools
{
   private static final String DEFAULT_FORMAT = getStringFormat(15, 12);

   /**
    * Asserts on a per component basis that the two rotation frame vectors are equal to an
    * {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected rotation frame vector. Not modified.
    * @param actual the actual rotation frame vector. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two rotation vectors are not equal or not expressed in the same
    *            reference frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertRotationFrameVectorEquals(FrameVector3DReadOnly expected, FrameVector3DReadOnly actual, double epsilon)
   {
      assertRotationFrameVectorEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two rotation frame vectors are equal to an
    * {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected rotation frame vector. Not modified.
    * @param actual the actual rotation frame vector. Not modified.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two rotation frame vectors are not equal or not expressed in the
    *            reference frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertRotationFrameVectorEquals(String messagePrefix, FrameVector3DReadOnly expected, FrameVector3DReadOnly actual, double epsilon)
   {
      assertRotationFrameVectorEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two rotation frame vectors are equal to an
    * {@code epsilon}.
    * <p>
    * The method returns {@code true} for angles such as:
    * {@code actualAngle = expectedAngle +/- 2.0 * Math.PI}.
    * </p>
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected rotation frame vector. Not modified.
    * @param actual the actual rotation frame vector. Not modified.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two rotation frame vectors are not equal or not expressed in the
    *            reference frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertRotationFrameVectorEquals(String messagePrefix, FrameVector3DReadOnly expected, FrameVector3DReadOnly actual, double epsilon,
                                                      String format)
   {
      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      assertRotationVectorEquals(expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected frame tuple.
    * @param actual the actual frame tuple.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two frame tuples are not equal or not expressed in the reference
    *            frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple2DEquals(FrameTuple2DReadOnly expected, FrameTuple2DReadOnly actual, double epsilon)
   {
      assertFrameTuple2DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected the expected frame tuple.
    * @param actual the actual frame tuple.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two frame tuples are not equal or not expressed in the reference
    *            frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple2DEquals(String messagePrefix, FrameTuple2DReadOnly expected, FrameTuple2DReadOnly actual, double epsilon)
   {
      assertFrameTuple2DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected the expected frame tuple.
    * @param actual the actual frame tuple.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two frame tuples are not equal or not expressed in the reference
    *            frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple2DEquals(String messagePrefix, FrameTuple2DReadOnly expected, FrameTuple2DReadOnly actual, double epsilon, String format)
   {
      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      assertTuple2DEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected frame tuple.
    * @param actual the actual frame tuple.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two frame tuples are not equal or not expressed in the reference
    *            frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple3DEquals(FrameTuple3DReadOnly expected, FrameTuple3DReadOnly actual, double epsilon)
   {
      assertFrameTuple3DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected frame tuple.
    * @param actual the actual frame tuple.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two frame tuples are not equal or not expressed in the reference
    *            frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple3DEquals(String messagePrefix, FrameTuple3DReadOnly expected, FrameTuple3DReadOnly actual, double epsilon)
   {
      assertFrameTuple3DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected frame tuple.
    * @param actual the actual frame tuple.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two frame tuples are not equal or not expressed in the reference
    *            frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple3DEquals(String messagePrefix, FrameTuple3DReadOnly expected, FrameTuple3DReadOnly actual, double epsilon, String format)
   {
      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      assertTuple3DEquals(messagePrefix, expected, actual, epsilon, format);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected frame tuple.
    * @param actual the actual frame tuple.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two tuples are not equal or not expressed in the reference
    *            frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple4DEquals(FrameTuple4DReadOnly expected, FrameTuple4DReadOnly actual, double epsilon)
   {
      assertFrameTuple4DEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected frame tuple.
    * @param actual the actual frame tuple.
    * @param epsilon the tolerance to use.
    * @throws AssertionError if the two tuples are not equal or not expressed in the reference
    *            frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple4DEquals(String messagePrefix, FrameTuple4DReadOnly expected, FrameTuple4DReadOnly actual, double epsilon)
   {
      assertFrameTuple4DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two frame tuples are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the error message.
    * @param expected the expected frame tuple.
    * @param actual the actual frame tuple.
    * @param epsilon the tolerance to use.
    * @param format the format to use for printing each component when an {@code AssertionError} is
    *           thrown.
    * @throws AssertionError if the two tuples are not equal or not expressed in the reference
    *            frame. If only one of the arguments is equal to {@code null}.
    */
   public static void assertFrameTuple4DEquals(String messagePrefix, FrameTuple4DReadOnly expected, FrameTuple4DReadOnly actual, double epsilon, String format)
   {
      if (expected.getReferenceFrame() != actual.getReferenceFrame())
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
      assertTuple4DEquals(messagePrefix, expected, actual, epsilon, format);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameTuple2DReadOnly expected, FrameTuple2DReadOnly actual, String format)
   {
      String expectedAsString = appendFrameName(getTuple2DString(format, expected), expected);
      String actualAsString = appendFrameName(getTuple2DString(format, actual), actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameTuple3DReadOnly expected, FrameTuple3DReadOnly actual, String format)
   {
      String expectedAsString = appendFrameName(getTuple3DString(format, expected), expected);
      String actualAsString = appendFrameName(getTuple3DString(format, actual), actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, FrameTuple4DReadOnly expected, FrameTuple4DReadOnly actual, String format)
   {
      String expectedAsString = appendFrameName(getTuple4DString(format, expected), expected);
      String actualAsString = appendFrameName(getTuple4DString(format, actual), actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, String expectedAsString, String actualAsString)
   {
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, null);
   }

   private static String appendFrameName(String string, ReferenceFrameHolder referenceFrameHolder)
   {
      return appendFrameName(string, referenceFrameHolder.getReferenceFrame());
   }

   private static String appendFrameName(String string, ReferenceFrame referenceFrame)
   {
      return string + "-" + referenceFrame;
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
