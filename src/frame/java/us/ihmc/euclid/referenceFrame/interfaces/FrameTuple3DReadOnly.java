package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Read-only interface for a 3D tuple expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Tuple3DReadOnly}, a {@link ReferenceFrame} is associated to
 * a {@code FrameTuple3DReadOnly}. This allows, for instance, to enforce, at runtime, that
 * operations on tuples occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameTuple3DReadOnly} extends {@code Tuple3DReadOnly}, it is compatible with
 * methods only requiring {@code Tuple3DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameTuple3DReadOnly}.
 * </p>
 */
public interface FrameTuple3DReadOnly extends Tuple3DReadOnly, EuclidFrameGeometry
{
   /**
    * Calculates the norm of the difference between {@code this} and {@code other}.
    * <p>
    * |{@code this} - {@code other}| = &radic;[({@code this.x} - {@code other.x})<sup>2</sup> +
    * ({@code this.y} - {@code other.y})<sup>2</sup> + ({@code this.z} - {@code other.z})<sup>2</sup>]
    * </p>
    *
    * @param other the other tuple to compare to. Not modified.
    * @return the norm squared of the difference.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default double differenceNorm(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Tuple3DReadOnly.super.differenceNorm(other);
   }

   /**
    * Calculates the norm squared of the difference between {@code this} and {@code other}.
    * <p>
    * |{@code this} - {@code other}|<sup>2</sup> = ({@code this.x} - {@code other.x})<sup>2</sup> +
    * ({@code this.y} - {@code other.y})<sup>2</sup> + ({@code this.z} - {@code other.z})<sup>2</sup>
    * </p>
    *
    * @param other the other tuple to compare to. Not modified.
    * @return the norm squared of the difference.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default double differenceNormSquared(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Tuple3DReadOnly.super.differenceNormSquared(other);
   }

   /**
    * Calculates and returns the value of the dot product of this tuple with {@code other}.
    * <p>
    * For instance, the dot product of two tuples p and q is defined as: <br>
    * p . q = &sum;<sub>i=1:3</sub>(p<sub>i</sub> * q<sub>i</sub>)
    * </p>
    *
    * @param other the other tuple used for the dot product. Not modified.
    * @return the value of the dot product.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default double dot(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Tuple3DReadOnly.super.dot(other);
   }

   /**
    * Gets a representative {@code String} of this tuple 3D given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String}
    * as follows:
    *
    * <pre>
    * (-0.558, -0.380,  0.130 ) - worldFrame
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidFrameIOTools.getFrameTuple3DString(format, this);
   }
}
