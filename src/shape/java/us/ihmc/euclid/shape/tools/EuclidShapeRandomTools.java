package us.ihmc.euclid.shape.tools;

import java.util.Collection;
import java.util.Random;

import us.ihmc.euclid.shape.Box3D;
import us.ihmc.euclid.shape.Capsule3D;
import us.ihmc.euclid.shape.Cylinder3D;
import us.ihmc.euclid.shape.Ellipsoid3D;
import us.ihmc.euclid.shape.PointShape3D;
import us.ihmc.euclid.shape.Ramp3D;
import us.ihmc.euclid.shape.Sphere3D;
import us.ihmc.euclid.shape.Torus3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class EuclidShapeRandomTools
{
   public static Box3D nextBox3D(Random random)
   {
      return nextBox3D(random, 0.0, 1.0);
   }

   public static Box3D nextBox3D(Random random, double minSize, double maxSize)
   {
      return new Box3D(EuclidCoreRandomTools.nextRigidBodyTransform(random), EuclidCoreRandomTools.nextDouble(random, minSize, maxSize),
                       EuclidCoreRandomTools.nextDouble(random, minSize, maxSize), EuclidCoreRandomTools.nextDouble(random, minSize, maxSize));
   }

   public static Capsule3D nextCapsule3D(Random random)
   {
      return nextCapsule3D(random, 0.0, 1.0, 0.0, 1.0);
   }

   public static Capsule3D nextCapsule3D(Random random, double minLength, double maxLength, double minRadius, double maxRadius)
   {
      return new Capsule3D(EuclidCoreRandomTools.nextRigidBodyTransform(random), EuclidCoreRandomTools.nextDouble(random, minLength, maxLength),
                           EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   public static Cylinder3D nextCylinder3D(Random random)
   {
      return nextCylinder3D(random, 0.0, 1.0, 0.0, 1.0);
   }

   public static Cylinder3D nextCylinder3D(Random random, double minLength, double maxLength, double minRadius, double maxRadius)
   {
      return new Cylinder3D(EuclidCoreRandomTools.nextRigidBodyTransform(random), EuclidCoreRandomTools.nextDouble(random, minLength, maxLength),
                            EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   public static Ellipsoid3D nextEllipsoid3D(Random random)
   {
      return nextEllipsoid3D(random, 0.0, 1.0);
   }

   public static Ellipsoid3D nextEllipsoid3D(Random random, double minRadius, double maxRadius)
   {
      return new Ellipsoid3D(EuclidCoreRandomTools.nextRigidBodyTransform(random), EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius),
                             EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius), EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   public static PointShape3D nextPointShape3D(Random random)
   {
      return new PointShape3D(EuclidCoreRandomTools.nextPoint3D(random));
   }

   public static PointShape3D nextPointShape3D(Random random, double minMax)
   {
      return new PointShape3D(EuclidCoreRandomTools.nextPoint3D(random, minMax));
   }

   public static Ramp3D nextRamp3D(Random random)
   {
      return nextRamp3D(random, 0.0, 1.0);
   }

   public static Ramp3D nextRamp3D(Random random, double minSize, double maxSize)
   {
      return new Ramp3D(EuclidCoreRandomTools.nextRigidBodyTransform(random), EuclidCoreRandomTools.nextDouble(random, minSize, maxSize),
                        EuclidCoreRandomTools.nextDouble(random, minSize, maxSize), EuclidCoreRandomTools.nextDouble(random, minSize, maxSize));
   }

   public static Sphere3D nextSphere3D(Random random)
   {
      return nextSphere3D(random, 0.0, 1.0);
   }

   public static Sphere3D nextSphere3D(Random random, double minRadius, double maxRadius)
   {
      return new Sphere3D(EuclidCoreRandomTools.nextPoint3D(random), EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   public static Torus3D nextTorus3D(Random random)
   {
      return nextTorus3D(random, 0.1, 1.0, 0.0, 0.1);
   }

   public static Torus3D nextTorus3D(Random random, double minRadius, double maxRadius, double minTubeRadius, double maxTubeRadius)
   {
      return new Torus3D(EuclidCoreRandomTools.nextRigidBodyTransform(random), EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius),
                         EuclidCoreRandomTools.nextDouble(random, minTubeRadius, maxTubeRadius));
   }

   public static Point2D nextPoint2DInTriangle(Random random, Point2DReadOnly a, Point2DReadOnly b, Point2DReadOnly c)
   {
      return new Point2D(nextPoint3DInTriangle(random, new Point3D(a), new Point3D(b), new Point3D(c)));
   }

   public static Point3D nextPoint3DInTriangle(Random random, Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c)
   {
      // Generating random point using the method introduced: http://mathworld.wolfram.com/TrianglePointPicking.html
      double alpha0 = random.nextDouble();
      double alpha1 = random.nextDouble();

      if (alpha0 + alpha1 > 1.0)
      { // The generated would be outside the triangle. Instead of discarding this point, we're folding the parallelogram.
         alpha0 = 1.0 - alpha0;
         alpha1 = 1.0 - alpha1;
      }

      Vector3D v0 = new Vector3D();
      Vector3D v1 = new Vector3D();
      v0.sub(b, a);
      v1.sub(c, a);

      Point3D next = new Point3D(a);
      next.scaleAdd(alpha0, v0, next);
      next.scaleAdd(alpha1, v1, next);

      return next;
   }

   public static Point3D nextPoint3DInTetrahedron(Random random, Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c, Point3DReadOnly d)
   {
      // Generating random point using the method introduced: http://vcg.isti.cnr.it/publications/papers/rndtetra_a.pdf
      double s = random.nextDouble();
      double t = random.nextDouble();
      double u = random.nextDouble();

      if (s + t > 1.0)
      {
         s = 1.0 - s;
         t = 1.0 - t;
      }

      if (s + t + u > 1.0)
      {
         if (t + u > 1.0)
         {
            double tOld = t;
            t = 1.0 - u;
            u = 1.0 - s - tOld;
         }
         else
         {
            double sOld = s;
            s = 1.0 - t - u;
            u = sOld + t + u - 1.0;
         }
      }

      Vector3D v0 = new Vector3D();
      Vector3D v1 = new Vector3D();
      Vector3D v2 = new Vector3D();
      v0.sub(b, a);
      v1.sub(c, a);
      v2.sub(d, a);

      Point3D next = new Point3D(a);
      next.scaleAdd(s, v0, next);
      next.scaleAdd(t, v1, next);
      next.scaleAdd(u, v2, next);

      return next;
   }

   // FIXME Generates points that are mostly sitting around the average of the points.
   public static Point3D nextWeightedAverage(Random random, Collection<? extends Point3DReadOnly> points)
   {
      return nextWeightedAverage(random, points.toArray(new Point3DReadOnly[points.size()]));
   }

   public static Point3D nextWeightedAverage(Random random, Point3DReadOnly[] points)
   {
      double[] weights = new double[points.length];
      double sum = 0.0;

      for (int j = 0; j < weights.length; j++)
         sum += weights[j] = random.nextDouble();

      Point3D next = new Point3D();

      for (int j = 0; j < weights.length; j++)
         next.scaleAdd(weights[j], points[j], next);

      next.scale(1.0 / sum);

      return next;
   }
}
