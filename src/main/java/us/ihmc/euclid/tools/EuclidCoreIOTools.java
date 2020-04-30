package us.ihmc.euclid.tools;

import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;
import java.util.function.Function;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

/**
 * {@code EuclidCoreIOTools} is intended to gather the input & output tools for printing, saving,
 * and loading geometry objects.
 * <p>
 * At this time, only a few print tools are offered, additional features will come in future
 * releases.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class EuclidCoreIOTools
{
   /** Default format used to print decimal numbers. */
   public static final String DEFAULT_FORMAT = getStringFormat(6, 3);

   private EuclidCoreIOTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Gets a representative {@code String} of {@code rigidBodyTransform} as follows:
    *
    * <pre>
    *  0.596  0.630  0.930 | -0.435
    * -0.264  0.763  0.575 | -0.464
    * -0.430 -0.188 -0.048 |  0.611
    *  0.000  0.000  0.000 |  1.000
    * </pre>
    *
    * @param rigidBodyTransform the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getRigidBodyTransformString(RigidBodyTransform rigidBodyTransform)
   {
      return getRigidBodyTransformString(DEFAULT_FORMAT, rigidBodyTransform);
   }

   /**
    * Gets a representative {@code String} of {@code rigidBodyTransform} given a specific format to
    * use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    *  0.596  0.630  0.930 | -0.435
    * -0.264  0.763  0.575 | -0.464
    * -0.430 -0.188 -0.048 |  0.611
    *  0.000  0.000  0.000 |  1.000
    * </pre>
    * </p>
    *
    * @param format             the format to use for each number.
    * @param rigidBodyTransform the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getRigidBodyTransformString(String format, RigidBodyTransform rigidBodyTransform)
   {
      if (rigidBodyTransform == null)
         return "null";
      else
         return getHomogeneousTransformString(format, rigidBodyTransform.getRotation(), rigidBodyTransform.getTranslation());
   }

   /**
    * Gets a representative {@code String} of {@code affineTransform} as follows:
    *
    * <pre>
    *  0.596  0.630  0.930 | -0.435
    * -0.264  0.763  0.575 | -0.464
    * -0.430 -0.188 -0.048 |  0.611
    *  0.000  0.000  0.000 |  1.000
    * </pre>
    *
    * @param affineTransform the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getAffineTransformString(AffineTransform affineTransform)
   {
      return getAffineTransformString(DEFAULT_FORMAT, affineTransform);
   }

   /**
    * Gets a representative {@code String} of {@code affineTransform} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    *  0.596  0.630  0.930 | -0.435
    * -0.264  0.763  0.575 | -0.464
    * -0.430 -0.188 -0.048 |  0.611
    *  0.000  0.000  0.000 |  1.000
    * </pre>
    * </p>
    *
    * @param format          the format to use for each number.
    * @param affineTransform the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getAffineTransformString(String format, AffineTransform affineTransform)
   {
      if (affineTransform == null)
         return "null";
      else
         return getHomogeneousTransformString(format, affineTransform.getRotationScaleMatrix(), affineTransform.getTranslationVector());
   }

   private static String getHomogeneousTransformString(String format, Matrix3DReadOnly matrix, Tuple3DReadOnly translation)
   {
      String ret = "";

      for (int i = 0; i < 3; i++)
      {
         ret += getStringOf(null, " ", " ", format, matrix.getElement(i, 0), matrix.getElement(i, 1), matrix.getElement(i, 2));
         ret += "| " + String.format(format, translation.getElement(i)) + "\n";
      }

      ret += getStringOf(null, " ", " ", format, 0.0, 0.0, 0.0);
      ret += "| " + String.format(format, 1.0);

      return ret;
   }

   /**
    * Gets a representative {@code String} of {@code quaternionBasedTransform} as follows:
    *
    * <pre>
    * Quaternion:  ( 0.174,  0.732, -0.222,  0.620 )
    * Translation: (-0.558, -0.380,  0.130 )
    * </pre>
    *
    * @param quaternionBasedTransform the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getQuaternionBasedTransformString(QuaternionBasedTransform quaternionBasedTransform)
   {
      return getQuaternionBasedTransformString(DEFAULT_FORMAT, quaternionBasedTransform);
   }

   /**
    * Gets a representative {@code String} of {@code quaternionBasedTransform} given a specific format
    * to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Quaternion:  ( 0.174,  0.732, -0.222,  0.620 )
    * Translation: (-0.558, -0.380,  0.130 )
    * </pre>
    * </p>
    *
    * @param format                   the format to use for each number.
    * @param quaternionBasedTransform the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getQuaternionBasedTransformString(String format, QuaternionBasedTransform quaternionBasedTransform)
   {
      if (quaternionBasedTransform == null)
         return "null";
      return getQuaternionBasedTransformString(format, quaternionBasedTransform.getRotation(), quaternionBasedTransform.getTranslation());
   }

   private static String getQuaternionBasedTransformString(String format, QuaternionReadOnly quaternion, Tuple3DReadOnly translation)
   {
      if (quaternion == null)
         return "null";

      String ret = "";

      ret += getStringOf("Quaternion:  (", " )\n", ", ", format, quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS());
      ret += getStringOf("Translation: (", " )", ", ", format, translation.getX(), translation.getY(), translation.getZ());
      return ret;
   }

   /**
    * Gets a representative {@code String} of {@code tuple} as follows:
    *
    * <pre>
    * (-0.675, -0.102 )
    * </pre>
    *
    * @param tuple the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTuple2DString(Tuple2DReadOnly tuple)
   {
      return getTuple2DString(DEFAULT_FORMAT, tuple);
   }

   /**
    * Gets a representative {@code String} of {@code tuple} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * (-0.675, -0.102 )
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param tuple  the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTuple2DString(String format, Tuple2DReadOnly tuple)
   {
      if (tuple == null)
         return "null";
      else
         return getStringOf("(", " )", ", ", format, tuple.getX(), tuple.getY());
   }

   /**
    * Gets a representative {@code String} of {@code tuple} as follows:
    *
    * <pre>
    * (-0.558, -0.380,  0.130 )
    * </pre>
    *
    * @param tuple the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTuple3DString(Tuple3DReadOnly tuple)
   {
      return getTuple3DString(DEFAULT_FORMAT, tuple);
   }

   /**
    * Gets a representative {@code String} of {@code tuple} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * (-0.558, -0.380,  0.130 )
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param tuple  the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTuple3DString(String format, Tuple3DReadOnly tuple)
   {
      if (tuple == null)
         return "null";
      else
         return getStringOf("(", " )", ", ", format, tuple.getX(), tuple.getY(), tuple.getZ());
   }

   /**
    * Gets a representative {@code String} of {@code tuple} as follows:
    *
    * <pre>
    * (-0.052, -0.173, -0.371,  0.087 )
    * </pre>
    *
    * @param tuple the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTuple4DString(Tuple4DReadOnly tuple)
   {
      return getTuple4DString(DEFAULT_FORMAT, tuple);
   }

   /**
    * Gets a representative {@code String} of {@code tuple} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * (-0.052, -0.173, -0.371,  0.087 )
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param tuple  the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTuple4DString(String format, Tuple4DReadOnly tuple)
   {
      if (tuple == null)
         return "null";
      else
         return getStringOf("(", " )", ", ", format, tuple.getX(), tuple.getY(), tuple.getZ(), tuple.getS());
   }

   /**
    * Gets a representative {@code String} of {@code axisAngle} as follows:
    *
    * <pre>
    * ( 0.674,  0.455,  0.582,  0.593 )
    * </pre>
    *
    * @param axisAngle the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getAxisAngleString(AxisAngleReadOnly axisAngle)
   {
      return getAxisAngleString(DEFAULT_FORMAT, axisAngle);
   }

   /**
    * Gets a representative {@code String} of {@code axisAngle} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * ( 0.674,  0.455,  0.582,  0.593 )
    * </pre>
    * </p>
    *
    * @param format    the format to use for each number.
    * @param axisAngle the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getAxisAngleString(String format, AxisAngleReadOnly axisAngle)
   {
      if (axisAngle == null)
         return "null";
      else
         return getStringOf("(", " )", ", ", format, axisAngle.getX(), axisAngle.getY(), axisAngle.getZ(), axisAngle.getAngle());
   }

   /**
    * Gets a representative {@code String} of {@code orientation2D} as follows:
    *
    * <pre>
    * (0.174)
    * </pre>
    *
    * @param orientation2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getOrientation2DString(Orientation2DReadOnly orientation2D)
   {
      return getOrientation2DString(DEFAULT_FORMAT, orientation2D);
   }

   /**
    * Gets a representative {@code String} of {@code orientation2D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * (0.174)
    * </pre>
    * </p>
    *
    * @param format        the format to use for each number.
    * @param orientation2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getOrientation2DString(String format, Orientation2DReadOnly orientation2D)
   {
      if (orientation2D == null)
         return "null";
      else
         return getOrientation2DString(format, orientation2D.getYaw());
   }

   /**
    * Gets a representative {@code String} of {@code orientation2DAngle} given a specific format to
    * use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * (0.174)
    * </pre>
    * </p>
    *
    * @param format             the format to use for each number.
    * @param orientation2DAngle the angle of the orientation 2D to get the {@code String} of. Not
    *                           modified.
    * @return the representative {@code String}.
    */
   public static String getOrientation2DString(String format, double orientation2DAngle)
   {
      return getStringOf("(", " )", ", ", format, orientation2DAngle);
   }

   /**
    * Gets a representative {@code String} of {@code matrix} as follows:
    *
    * <pre>
    * /-0.576, -0.784,  0.949 \
    * | 0.649, -0.542, -0.941 |
    * \-0.486, -0.502, -0.619 /
    * </pre>
    *
    * @param matrix the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getMatrix3DString(Matrix3DReadOnly matrix)
   {
      return getMatrix3DString(DEFAULT_FORMAT, matrix);
   }

   /**
    * Gets a representative {@code String} of {@code matrix} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * /-0.576, -0.784,  0.949 \
    * | 0.649, -0.542, -0.941 |
    * \-0.486, -0.502, -0.619 /
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param matrix the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getMatrix3DString(String format, Matrix3DReadOnly matrix)
   {
      if (matrix == null)
         return "null";
      else
         return getMatrix3DString(format,
                                  matrix.getM00(),
                                  matrix.getM01(),
                                  matrix.getM02(),
                                  matrix.getM10(),
                                  matrix.getM11(),
                                  matrix.getM12(),
                                  matrix.getM20(),
                                  matrix.getM21(),
                                  matrix.getM22());
   }

   /**
    * Gets a representative {@code String} of {@code matrix} as follows:
    *
    * <pre>
    * /-0.576, -0.784,  0.949 \
    * | 0.649, -0.542, -0.941 |
    * \-0.486, -0.502, -0.619 /
    * </pre>
    * </p>
    *
    * @param m00 the 1st row 1st column coefficient of the matrix.
    * @param m01 the 1st row 2nd column coefficient of the matrix.
    * @param m02 the 1st row 3rd column coefficient of the matrix.
    * @param m10 the 2nd row 1st column coefficient of the matrix.
    * @param m11 the 2nd row 2nd column coefficient of the matrix.
    * @param m12 the 2nd row 3rd column coefficient of the matrix.
    * @param m20 the 3rd row 1st column coefficient of the matrix.
    * @param m21 the 3rd row 2nd column coefficient of the matrix.
    * @param m22 the 3rd row 3rd column coefficient of the matrix.
    * @return the representative {@code String}.
    */
   public static String getMatrix3DString(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      return getMatrix3DString(DEFAULT_FORMAT, m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Gets a representative {@code String} of {@code matrix} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * /-0.576, -0.784,  0.949 \
    * | 0.649, -0.542, -0.941 |
    * \-0.486, -0.502, -0.619 /
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param m00    the 1st row 1st column coefficient of the matrix.
    * @param m01    the 1st row 2nd column coefficient of the matrix.
    * @param m02    the 1st row 3rd column coefficient of the matrix.
    * @param m10    the 2nd row 1st column coefficient of the matrix.
    * @param m11    the 2nd row 2nd column coefficient of the matrix.
    * @param m12    the 2nd row 3rd column coefficient of the matrix.
    * @param m20    the 3rd row 1st column coefficient of the matrix.
    * @param m21    the 3rd row 2nd column coefficient of the matrix.
    * @param m22    the 3rd row 3rd column coefficient of the matrix.
    * @return the representative {@code String}.
    */
   public static String getMatrix3DString(String format, double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21,
                                          double m22)
   {
      String ret = getStringOf("/", " \\\n", ", ", format, m00, m01, m02);
      ret += getStringOf("|", " |\n", ", ", format, m10, m11, m12);
      ret += getStringOf("\\", " /", ", ", format, m20, m21, m22);
      return ret;
   }

   /**
    * Gets a representative {@code String} of {@code yawPitchRoll} as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 )
    * </pre>
    *
    * @param yawPitchRoll the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getYawPitchRollString(YawPitchRollReadOnly yawPitchRoll)
   {
      return getYawPitchRollString(DEFAULT_FORMAT, yawPitchRoll);
   }

   /**
    * Gets a representative {@code String} of {@code yawPitchRoll} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 )
    * </pre>
    * </p>
    *
    * @param format       the format to use for each number.
    * @param yawPitchRoll the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getYawPitchRollString(String format, YawPitchRollReadOnly yawPitchRoll)
   {
      if (yawPitchRoll == null)
         return "null";
      else
         return getYawPitchRollString(format, yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll());
   }

   /**
    * Gets a representative {@code String} of {@code orientation} using a yaw-pitch-roll representation
    * as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 )
    * </pre>
    *
    * @param orientation the orientation to get the {@code String} of using a yaw-pitch-roll
    *                    representation. Not modified.
    * @return the representative {@code String}.
    */
   public static String getStringAsYawPitchRoll(Orientation3DReadOnly orientation)
   {
      return getStringAsYawPitchRoll(DEFAULT_FORMAT, orientation);
   }

   /**
    * Gets a representative {@code String} of {@code orientation} using a yaw-pitch-roll representation
    * and given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 )
    * </pre>
    * </p>
    *
    * @param format      the format to use for each number.
    * @param orientation the orientation to get the {@code String} of using a yaw-pitch-roll
    *                    representation. Not modified.
    * @return the representative {@code String}.
    */
   public static String getStringAsYawPitchRoll(String format, Orientation3DReadOnly orientation)
   {
      if (orientation == null)
         return "null";
      else
         return getYawPitchRollString(format, orientation.getYaw(), orientation.getPitch(), orientation.getRoll());
   }

   /**
    * Gets a representative {@code String} of {@code yawPitchRoll} as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 )
    * </pre>
    *
    * @param yaw   the first angle representing the rotation around the z-axis.
    * @param pitch the second angle representing the rotation around the y-axis.
    * @param roll  the third angle representing the rotation around the x-axis.
    * @return the representative {@code String}.
    */
   public static String getYawPitchRollString(double yaw, double pitch, double roll)
   {
      return getYawPitchRollString(DEFAULT_FORMAT, yaw, pitch, roll);
   }

   /**
    * Gets a representative {@code String} of {@code yawPitchRoll} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 )
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param yaw    the first angle representing the rotation around the z-axis.
    * @param pitch  the second angle representing the rotation around the y-axis.
    * @param roll   the third angle representing the rotation around the x-axis.
    * @return the representative {@code String}.
    */
   public static String getYawPitchRollString(String format, double yaw, double pitch, double roll)
   {
      return getStringOf("yaw-pitch-roll: (", ")", ", ", yaw, pitch, roll);
   }

   /**
    * Gets a representative {@code String} of a series of doubles given specific prefix, suffix, and
    * separator.
    * <p>
    * Using {@code prefix = "("}, {@code suffix = ")"}, and {@code separator = ", "}, this provides a
    * {@code String} as follows:
    *
    * <pre>
    * ( 0.123, -0.480,  1.457)
    * </pre>
    * </p>
    *
    * @param prefix    the {@code String} to prepend to the result.
    * @param suffix    the {@code String} to append to the result.
    * @param separator the {@code String} to insert between two values.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String prefix, String suffix, String separator, double... values)
   {
      return getStringOf(prefix, suffix, separator, DEFAULT_FORMAT, values);
   }

   /**
    * Gets a representative {@code String} of a series of doubles given specific prefix, suffix,
    * separator, and format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, {@code prefix = "("}, {@code suffix = ")"}, and
    * {@code separator = ", "}, this provides a {@code String} as follows:
    *
    * <pre>
    * ( 0.123, -0.480,  1.457)
    * </pre>
    * </p>
    *
    * @param prefix    the {@code String} to prepend to the result.
    * @param suffix    the {@code String} to append to the result.
    * @param separator the {@code String} to insert between two values.
    * @param format    the format to use for each number.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String prefix, String suffix, String separator, String format, double... values)
   {
      if (values == null)
         return "null";

      String ret = getStringOf(separator, format, values);

      if (prefix != null)
         ret = prefix + ret;

      if (suffix != null)
         ret += suffix;

      return ret;
   }

   /**
    * Gets a representative {@code String} of a series of doubles given specific prefix, suffix,
    * separator, and format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT} and {@code separator = ", "}, this provides a
    * {@code String} as follows:
    *
    * <pre>
    *  0.123, -0.480,  1.457
    * </pre>
    * </p>
    *
    * @param separator the {@code String} to insert between two values.
    * @param format    the format to use for each number.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String separator, String format, double... values)
   {
      if (values == null)
         return "null";

      if (values.length == 0)
         return "";
      String ret = String.format(format, values[0]);
      for (int i = 1; i < values.length; i++)
         ret += separator + String.format(format, values[i]);
      return ret;
   }

   /**
    * Gets a representative {@code String} of the elements contained in the given array.
    * <p>
    * This provides an alternative to {@link Arrays#toString(Object[])} where the format of the output
    * can be controlled by defining a custom {@code separator}. For instance, with
    * {@code separator = \n} the resulting {@code String} is composed of one element per line as
    * opposed to {@link Arrays#toString(Object[])} which outputs all the elements in one line.
    * </p>
    *
    * @param prefix                  the {@code String} to prepend to the result.
    * @param suffix                  the {@code String} to append to the result.
    * @param separator               the {@code String} used to separate elements of the array.
    * @param array                   the array of elements to get the {@code String} of.
    * @param elementToStringFunction the {@code Function} used to generate a representative
    *                                {@code String} for each element.
    * @return the representative {@code String}.
    */
   public static <T> String getArrayString(String prefix, String suffix, String separator, T[] array, Function<T, String> elementToStringFunction)
   {
      if (array == null)
         return "null";
      else
         return getCollectionString(prefix, suffix, separator, Arrays.asList(array), elementToStringFunction);
   }

   /**
    * Gets a representative {@code String} of the elements contained in the given array.
    * <p>
    * This provides an alternative to {@link Arrays#toString(Object[])} where the format of the output
    * can be controller by defining a custom {@code separator}. For instance, with
    * {@code separator = \n} the resulting {@code String} is composed of one element per line as
    * opposed to {@link Arrays#toString(Object[])} which outputs all the elements in one line.
    * </p>
    *
    * @param separator               the {@code String} used to separate elements of the array.
    * @param array                   the array of elements to get the {@code String} of.
    * @param elementToStringFunction the {@code Function} used to generate a representative
    *                                {@code String} for each element.
    * @return the representative {@code String}.
    */
   public static <T> String getArrayString(String separator, T[] array, Function<T, String> elementToStringFunction)
   {
      return getCollectionString(separator, Arrays.asList(array), elementToStringFunction);
   }

   /**
    * Gets a representative {@code String} of the elements contained in the given {@code Collection}.
    * <p>
    * This provides an alternative to {@link Collection#toString()} where the format of the output can
    * be controller by defining a custom {@code separator}. For instance, with {@code separator = \n}
    * the resulting {@code String} is composed of one element per line as opposed to
    * {@link Collection#toString()} which outputs all the elements in one line.
    * </p>
    *
    * @param prefix                  the {@code String} to prepend to the result.
    * @param suffix                  the {@code String} to append to the result.
    * @param separator               the {@code String} used to separate elements of the collection.
    * @param collection              the series of elements to get the {@code String} of.
    * @param elementToStringFunction the {@code Function} used to generate a representative
    *                                {@code String} for each element.
    * @return the representative {@code String}.
    */
   public static <T> String getCollectionString(String prefix, String suffix, String separator, Collection<? extends T> collection,
                                                Function<T, String> elementToStringFunction)
   {
      if (collection == null)
         return "null";

      String ret = getCollectionString(separator, collection, elementToStringFunction);

      if (prefix != null)
         ret = prefix + ret;

      if (suffix != null)
         ret += suffix;

      return ret;
   }

   /**
    * Gets a representative {@code String} of the elements contained in the given {@code Collection}.
    * <p>
    * This provides an alternative to {@link Collection#toString()} where the format of the output can
    * be controller by defining a custom {@code separator}. For instance, with {@code separator = \n}
    * the resulting {@code String} is composed of one element per line as opposed to
    * {@link Collection#toString()} which outputs all the elements in one line.
    * </p>
    *
    * @param separator               the {@code String} used to separate elements of the collection.
    * @param collection              the series of elements to get the {@code String} of.
    * @param elementToStringFunction the {@code Function} used to generate a representative
    *                                {@code String} for each element.
    * @return the representative {@code String}.
    */
   public static <T> String getCollectionString(String separator, Collection<? extends T> collection, Function<T, String> elementToStringFunction)
   {
      if (collection == null)
         return "null";
      if (collection.isEmpty())
         return "";

      Iterator<? extends T> iterator = collection.iterator();
      String ret = elementToStringFunction.apply(iterator.next());
      while (iterator.hasNext())
         ret += separator + elementToStringFunction.apply(iterator.next());
      return ret;
   }

   /**
    * Gets the {@code String} for formatting decimal numbers.
    *
    * @param numberOfChar the number of character to print.
    * @param precision    the precision of the output.
    * @return the formatting {@code String}.
    */
   public static String getStringFormat(int numberOfChar, int precision)
   {
      return "%" + numberOfChar + "." + precision + "f";
   }
}
