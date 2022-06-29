package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

/**
 * Read-only interface for a 4D tuple expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Tuple4DReadOnly}, a {@link ReferenceFrame} is associated to
 * a {@code FrameTuple4DReadOnly}. This allows, for instance, to enforce, at runtime, that
 * operations on tuples occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameTuple4DReadOnly} extends {@code Tuple4DReadOnly}, it is compatible with
 * methods only requiring {@code Tuple4DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameTuple4DReadOnly}.
 * </p>
 */
public interface FrameTuple4DReadOnly extends Tuple4DReadOnly, EuclidFrameGeometry
{
   /**
    * Calculates the norm of the difference between {@code this} and {@code other}.
    * <p>
    * |{@code this} - {@code other}| = &radic;[({@code this.x} - {@code other.x})<sup>2</sup> +
    * ({@code this.y} - {@code other.y})<sup>2</sup> + ({@code this.z} - {@code other.z})<sup>2</sup> +
    * ({@code this.s} - {@code other.s})<sup>2</sup>]
    * </p>
    *
    * @param other the other tuple to compare to. Not modified.
    * @return the norm squared of the difference.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default double differenceNorm(FrameTuple4DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Tuple4DReadOnly.super.differenceNorm(other);
   }

   /**
    * Calculates the norm squared of the difference between {@code this} and {@code other}.
    * <p>
    * |{@code this} - {@code other}|<sup>2</sup> = ({@code this.x} - {@code other.x})<sup>2</sup> +
    * ({@code this.y} - {@code other.y})<sup>2</sup> + ({@code this.z} - {@code other.z})<sup>2</sup>
    * +({@code this.s} - {@code other.s})<sup>2</sup>
    * </p>
    *
    * @param other the other tuple to compare to. Not modified.
    * @return the norm squared of the difference.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default double differenceNormSquared(FrameTuple4DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Tuple4DReadOnly.super.differenceNormSquared(other);
   }

   /**
    * Calculates and returns the value of the dot product of this tuple with {@code other}.
    * <p>
    * For instance, the dot product of two tuples p and q is defined as: <br>
    * p . q = &sum;<sub>i=1:4</sub>(p<sub>i</sub> * q<sub>i</sub>)
    * </p>
    *
    * @param other the other vector used for the dot product. Not modified.
    * @return the value of the dot product.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default double dot(FrameTuple4DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Tuple4DReadOnly.super.dot(other);
   }

   /**
    * Gets a representative {@code String} of this tuple 4D given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String}
    * as follows:
    *
    * <pre>
    * (-0.052, -0.173, -0.371,  0.087 ) - worldFrame
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidFrameIOTools.getFrameTuple4DString(format, this);
   }
}
