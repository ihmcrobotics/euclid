package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;

/**
 * Base interface that represents any geometry object that is expressed in a reference frame.
 */
public interface EuclidFrameGeometry extends EuclidGeometry, ReferenceFrameHolder
{
   /**
    * Tests if {@code this} is exactly equal to the given {@code geometry}.
    * <p>
    * The 2 geometry objects can only be considered equal if they are expressed in the same reference
    * frame.
    * </p>
    * <p>
    * The test is achieved on a per component basis. A failing test does not necessarily mean that the
    * two objects are different in a geometric way.
    * </p>
    *
    * @param frameGeometry the geometry to compare against {@code this}. Not modified.
    * @return {@code true} if the two objects are of same type and are equal component-wise,
    *         {@code false} otherwise.
    */
   default boolean equals(EuclidFrameGeometry frameGeometry)
   {
      if (frameGeometry == this)
         return true;
      if (frameGeometry == null)
         return false;
      if (getReferenceFrame() != frameGeometry.getReferenceFrame())
         return false;

      return equals((EuclidGeometry) frameGeometry);
   }

   /**
    * Tests if {@code this} is approximately equal to {@code geometry} using the tolerance
    * {@code epsilon}.
    * <p>
    * The 2 geometry objects can only be considered equal if they are expressed in the same reference
    * frame.
    * </p>
    * <p>
    * Similar to {@link #equals(EuclidFrameGeometry)}, the test is achieved on a per component basis. A
    * failing test does not necessarily mean that the two objects are different in a geometric way.
    * </p>
    *
    * @param frameGeometry the geometry to compare against {@code this}. Not modified.
    * @param epsilon       tolerance to use when comparing each component.
    * @return {@code true} if the two objects are of same type and are approximately equal
    *         component-wise, {@code false} otherwise.
    */
   default boolean epsilonEquals(EuclidFrameGeometry frameGeometry, double epsilon)
   {
      if (frameGeometry == this)
         return true;
      if (frameGeometry == null)
         return false;
      if (getReferenceFrame() != frameGeometry.getReferenceFrame())
         return false;
      return epsilonEquals((EuclidGeometry) frameGeometry, epsilon);
   }

   /**
    * Tests if {@code this} and {@code geometry} represent the same geometry to an {@code epsilon}.
    * <p>
    * The 2 geometry objects can only be considered equal if they are expressed in the same reference
    * frame.
    * </p>
    * <p>
    * The implementation of this test depends on the type of geometry. For instance, two points will be
    * considered geometrically equal if they are at a distance from each other that is less or equal
    * than {@code epsilon}. The two object must represent the same type of geometry.
    * </p>
    *
    * @param frameGeometry the geometry to compare against {@code this}. Not modified.
    * @param epsilon       tolerance to use when comparing the two geometries, usually refers to a
    *                      distance.
    * @return {@code true} if the two objects represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(EuclidFrameGeometry frameGeometry, double epsilon)
   {
      if (frameGeometry == this)
         return true;
      if (frameGeometry == null)
         return false;
      if (getReferenceFrame() != frameGeometry.getReferenceFrame())
         return false;
      return geometricallyEquals((EuclidGeometry) frameGeometry, epsilon);
   }
}
