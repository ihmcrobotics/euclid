package us.ihmc.euclid.shape.collision.shapeModifier;

import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;

public abstract class SphereTorusPatchesBoundingVolume<S extends Shape3DReadOnly> implements SupportingVertexHolder
{
   protected double minimumMargin, maximumMargin;
   protected double largeRadius, smallRadius;
   protected S shape3D;

   public SphereTorusPatchesBoundingVolume()
   {
   }

   public void setMargins(double minimumMargin, double maximumMargin)
   {
      if (maximumMargin <= minimumMargin)
         throw new IllegalArgumentException("The maximum margin has to be strictly grater that the minimum margin, max margin: " + maximumMargin
               + ", min margin: " + minimumMargin);
      this.minimumMargin = minimumMargin;
      this.maximumMargin = maximumMargin;
   }

   public void setShape3D(S shape3D)
   {
      this.shape3D = shape3D;
      updateRadii();
   }

   protected abstract double findMaximumEdgeLengthSquared();

   /**
    * <pre>
    * r = h
    *      r^2 - g^2 - 0.25 * l<sub>max</sub>
    * R = ------------------------
    *           2 * (r - g)
    * </pre>
    * 
    * where:
    * <ul>
    * <li><tt>R</tt> is {@link #largeRadius}
    * <li><tt>r</tt> is {@link #smallRadius}
    * <li><tt>h</tt> is {@link #minimumMargin}
    * <li><tt>g</tt> is {@link #maximumMargin}
    * <li><tt>l<sub>max</max></tt> is the maximum edge length that needs to be covered by the large
    * bounding sphere.
    * </ul>
    */
   protected void updateRadii()
   {
      double safeMaximumMargin = maximumMargin;
      double maximumEdgeLengthSquared = findMaximumEdgeLengthSquared();

      if (EuclidCoreTools.square(maximumMargin - minimumMargin) > 0.25 * maximumEdgeLengthSquared)
      {
         safeMaximumMargin = 0.99 * (0.5 * EuclidCoreTools.squareRoot(maximumEdgeLengthSquared) + minimumMargin);
         System.err.println(getClass().getSimpleName() + ": Unachievable margins, modified maximumMargin from:" + maximumMargin + ", down to: "
               + safeMaximumMargin);
      }

      smallRadius = minimumMargin;
      double smallRadiusSquared = EuclidCoreTools.square(smallRadius);
      double maximumMarginSquared = EuclidCoreTools.square(safeMaximumMargin);
      largeRadius = smallRadiusSquared - maximumMarginSquared - 0.25 * maximumEdgeLengthSquared;
      largeRadius /= 2.0 * (smallRadius - safeMaximumMargin);

      System.out.println("largeRadius: " + largeRadius + ", g = "
            + (largeRadius - Math.sqrt(EuclidCoreTools.square(largeRadius - smallRadius) - 0.25 * maximumEdgeLengthSquared)));
   }

   public double getMinimumMargin()
   {
      return minimumMargin;
   }

   public double getMaximumMargin()
   {
      return maximumMargin;
   }

   public double getSmallRadius()
   {
      return smallRadius;
   }

   public double getLargeRadius()
   {
      return largeRadius;
   }

   public S getShape3D()
   {
      return shape3D;
   }
}
