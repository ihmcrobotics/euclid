package us.ihmc.euclid.geometry.tools;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.ObjDoubleConsumer;

import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DBasics;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * This class provides a varieties of factories to create Euclid geometry types.
 *
 * @author Sylvain Bertrand
 */
public class EuclidGeometryFactories
{
   /**
    * Creates a new bounding box 2D that is a read-only view of the given min and max positions.
    *
    * @param minPoint the position to use for representing minimum bound.
    * @param maxPoint the position to use for representing maximum bound.
    * @return the new read-only bounding box 2D.
    */
   public static BoundingBox2DReadOnly newLinkedBoundingBox2DReadOnly(Point2DReadOnly minPoint, Point2DReadOnly maxPoint)
   {
      return new BoundingBox2DReadOnly()
      {
         @Override
         public Point2DReadOnly getMinPoint()
         {
            return minPoint;
         }

         @Override
         public Point2DReadOnly getMaxPoint()
         {
            return maxPoint;
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof BoundingBox2DReadOnly)
               return equals((BoundingBox2DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
         }

         @Override
         public int hashCode()
         {
            return EuclidHashCodeTools.toIntHashCode(minPoint, maxPoint);
         }
      };
   }

   /**
    * Creates a new bounding box 3D that is a read-only view of the given min and max positions.
    *
    * @param minPoint the position to use for representing minimum bound.
    * @param maxPoint the position to use for representing maximum bound.
    * @return the new read-only bounding box 3D.
    */
   public static BoundingBox3DReadOnly newLinkedBoundingBox3DReadOnly(Point3DReadOnly minPoint, Point3DReadOnly maxPoint)
   {
      return new BoundingBox3DReadOnly()
      {
         @Override
         public Point3DReadOnly getMinPoint()
         {
            return minPoint;
         }

         @Override
         public Point3DReadOnly getMaxPoint()
         {
            return maxPoint;
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof BoundingBox3DReadOnly)
               return equals((BoundingBox3DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
         }

         @Override
         public int hashCode()
         {
            return EuclidHashCodeTools.toIntHashCode(minPoint, maxPoint);
         }
      };
   }

   /**
    * Creates a new bounding box 2D backed by the given min and max positions.
    *
    * @param minPoint the position to use for representing minimum bound.
    * @param maxPoint the position to use for representing maximum bound.
    * @return the new linked bounding box 2D.
    */
   public static BoundingBox2DBasics newLinkedBoundingBox2DBasics(Point2DBasics minPoint, Point2DBasics maxPoint)
   {
      return new BoundingBox2DBasics()
      {
         @Override
         public Point2DBasics getMinPoint()
         {
            return minPoint;
         }

         @Override
         public Point2DBasics getMaxPoint()
         {
            return maxPoint;
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof BoundingBox2DReadOnly)
               return equals((BoundingBox2DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
         }

         @Override
         public int hashCode()
         {
            return EuclidHashCodeTools.toIntHashCode(minPoint, maxPoint);
         }
      };
   }

   /**
    * Creates a new bounding box 3D backed by the given min and max positions.
    *
    * @param minPoint the position to use for representing minimum bound.
    * @param maxPoint the position to use for representing maximum bound.
    * @return the new linked bounding box 3D.
    */
   public static BoundingBox3DBasics newLinkedBoundingBox3DBasics(Point3DBasics minPoint, Point3DBasics maxPoint)
   {
      return new BoundingBox3DBasics()
      {
         @Override
         public Point3DBasics getMinPoint()
         {
            return minPoint;
         }

         @Override
         public Point3DBasics getMaxPoint()
         {
            return maxPoint;
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof BoundingBox3DReadOnly)
               return equals((BoundingBox3DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
         }

         @Override
         public int hashCode()
         {
            return EuclidHashCodeTools.toIntHashCode(minPoint, maxPoint);
         }
      };
   }

   /**
    * Creates a new bounding box that can be used to observe read and write operations.
    *
    * @param valueChangedListener  the listener to be notified whenever a component of the bounding box
    *                              has been modified. The corresponding constants {@link Axis2D} and
    *                              {@link Bound} will be passed to indicate the component that was
    *                              changed alongside its new value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the bounding box
    *                              is being accessed. The corresponding constants {@link Axis2D} and
    *                              {@link Bound} will be passed to indicate the component being
    *                              accessed. Can be {@code null}.
    * @return the observable bounding box.
    */
   public static BoundingBox2DBasics newObservableBoundingBox2DBasics(BoundingBoxChangedListener<Axis2D> valueChangedListener,
                                                                      BiConsumer<Axis2D, Bound> valueAccessedListener)
   {
      return newObservableBoundingBox2DBasics(valueChangedListener, valueAccessedListener, new BoundingBox2D());
   }

   /**
    * Creates a linked bounding box that can be used to observe read and write operations on the
    * source.
    *
    * @param valueChangedListener  the listener to be notified whenever a component of the bounding box
    *                              has been modified. The corresponding constants {@link Axis2D} and
    *                              {@link Bound} will be passed to indicate the component that was
    *                              changed alongside its new value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the bounding box
    *                              is being accessed. The corresponding constants {@link Axis2D} and
    *                              {@link Bound} will be passed to indicate the component being
    *                              accessed. Can be {@code null}.
    * @param source                the original bounding box to link and observe. Modifiable via the
    *                              linked point interface.
    * @return the observable bounding box.
    */
   public static BoundingBox2DBasics newObservableBoundingBox2DBasics(BoundingBoxChangedListener<Axis2D> valueChangedListener,
                                                                      BiConsumer<Axis2D, Bound> valueAccessedListener,
                                                                      BoundingBox2DBasics source)
   {
      return new BoundingBox2DBasics()
      {
         private final Point2DBasics minPoint, maxPoint;
         private boolean isNotifying = false;

         {
            ObjDoubleConsumer<Axis2D> minChangedListener = toPoint2DValueChangedListener(valueChangedListener, Bound.MIN);
            Consumer<Axis2D> minAccessedListener = toPoint2DValueAccessedListener(valueAccessedListener, Bound.MIN);
            ObjDoubleConsumer<Axis2D> maxChangedListener = toPoint2DValueChangedListener(valueChangedListener, Bound.MAX);
            Consumer<Axis2D> maxAccessedListener = toPoint2DValueAccessedListener(valueAccessedListener, Bound.MAX);

            minPoint = EuclidCoreFactories.newObservablePoint2DBasics(minChangedListener, minAccessedListener, source.getMinPoint());
            maxPoint = EuclidCoreFactories.newObservablePoint2DBasics(maxChangedListener, maxAccessedListener, source.getMaxPoint());
         }

         private ObjDoubleConsumer<Axis2D> toPoint2DValueChangedListener(BoundingBoxChangedListener<Axis2D> valueChangedListener, Bound bound)
         {
            if (valueChangedListener == null)
               return null;
            else
               return (axis, newValue) ->
               {
                  if (isNotifying)
                     return;
                  isNotifying = true;
                  valueChangedListener.changed(axis, bound, newValue);
                  isNotifying = false;
               };
         }

         private Consumer<Axis2D> toPoint2DValueAccessedListener(BiConsumer<Axis2D, Bound> valueAccessedListener, Bound bound)
         {
            if (valueAccessedListener == null)
               return null;
            else
               return axis ->
               {
                  if (isNotifying)
                     return;
                  isNotifying = true;
                  valueAccessedListener.accept(axis, bound);
                  isNotifying = false;
               };
         }

         @Override
         public Point2DBasics getMinPoint()
         {
            return minPoint;
         }

         @Override
         public Point2DBasics getMaxPoint()
         {
            return maxPoint;
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof BoundingBox2DReadOnly)
               return equals((BoundingBox2DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
         }

         @Override
         public int hashCode()
         {
            return EuclidHashCodeTools.toIntHashCode(minPoint, maxPoint);
         }
      };
   }

   /**
    * Creates a new bounding box that can be used to observe read and write operations.
    *
    * @param valueChangedListener  the listener to be notified whenever a component of the bounding box
    *                              has been modified. The corresponding constants {@link Axis3D} and
    *                              {@link Bound} will be passed to indicate the component that was
    *                              changed alongside its new value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the bounding box
    *                              is being accessed. The corresponding constants {@link Axis3D} and
    *                              {@link Bound} will be passed to indicate the component being
    *                              accessed. Can be {@code null}.
    * @return the observable bounding box.
    */
   public static BoundingBox3DBasics newObservableBoundingBox3DBasics(BoundingBoxChangedListener<Axis3D> valueChangedListener,
                                                                      BiConsumer<Axis3D, Bound> valueAccessedListener)
   {
      return newObservableBoundingBox3DBasics(valueChangedListener, valueAccessedListener, new BoundingBox3D());
   }

   /**
    * Creates a linked bounding box that can be used to observe read and write operations on the
    * source.
    *
    * @param valueChangedListener  the listener to be notified whenever a component of the bounding box
    *                              has been modified. The corresponding constants {@link Axis3D} and
    *                              {@link Bound} will be passed to indicate the component that was
    *                              changed alongside its new value. Can be {@code null}.
    * @param valueAccessedListener the listener to be notified whenever a component of the bounding box
    *                              is being accessed. The corresponding constants {@link Axis3D} and
    *                              {@link Bound} will be passed to indicate the component being
    *                              accessed. Can be {@code null}.
    * @param source                the original bounding box to link and observe. Modifiable via the
    *                              linked point interface.
    * @return the observable bounding box.
    */
   public static BoundingBox3DBasics newObservableBoundingBox3DBasics(BoundingBoxChangedListener<Axis3D> valueChangedListener,
                                                                      BiConsumer<Axis3D, Bound> valueAccessedListener,
                                                                      BoundingBox3DBasics source)
   {
      return new BoundingBox3DBasics()
      {
         private final Point3DBasics minPoint, maxPoint;
         private boolean isNotifying = false;

         {
            ObjDoubleConsumer<Axis3D> minChangedListener = toPoint3DValueChangedListener(valueChangedListener, Bound.MIN);
            Consumer<Axis3D> minAccessedListener = toPoint3DValueAccessedListener(valueAccessedListener, Bound.MIN);
            ObjDoubleConsumer<Axis3D> maxChangedListener = toPoint3DValueChangedListener(valueChangedListener, Bound.MAX);
            Consumer<Axis3D> maxAccessedListener = toPoint3DValueAccessedListener(valueAccessedListener, Bound.MAX);

            minPoint = EuclidCoreFactories.newObservablePoint3DBasics(minChangedListener, minAccessedListener, source.getMinPoint());
            maxPoint = EuclidCoreFactories.newObservablePoint3DBasics(maxChangedListener, maxAccessedListener, source.getMaxPoint());
         }

         private ObjDoubleConsumer<Axis3D> toPoint3DValueChangedListener(BoundingBoxChangedListener<Axis3D> valueChangedListener, Bound bound)
         {
            if (valueChangedListener == null)
               return null;
            else
               return (axis, newValue) ->
               {
                  if (isNotifying)
                     return;
                  isNotifying = true;
                  valueChangedListener.changed(axis, bound, newValue);
                  isNotifying = false;
               };
         }

         private Consumer<Axis3D> toPoint3DValueAccessedListener(BiConsumer<Axis3D, Bound> valueAccessedListener, Bound bound)
         {
            if (valueAccessedListener == null)
               return null;
            else
               return axis ->
               {
                  if (isNotifying)
                     return;
                  isNotifying = true;
                  valueAccessedListener.accept(axis, bound);
                  isNotifying = false;
               };
         }

         @Override
         public Point3DBasics getMinPoint()
         {
            return minPoint;
         }

         @Override
         public Point3DBasics getMaxPoint()
         {
            return maxPoint;
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof BoundingBox3DReadOnly)
               return equals((BoundingBox3DReadOnly) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
         }

         @Override
         public int hashCode()
         {
            return EuclidHashCodeTools.toIntHashCode(minPoint, maxPoint);
         }
      };
   }

   /**
    * Functional interface for implementing listener that is to be notified whenever the bounding box
    * its attached is being modified.
    *
    * @author Sylvain Bertrand
    * @param <Axis> The 2D or 3D axis type that depends on the bounding box to listen to.
    */
   @FunctionalInterface
   public static interface BoundingBoxChangedListener<Axis>
   {
      /**
       * Notifies that the bounding box has changed.
       *
       * @param axis     the axis along which the bounding box was changed.
       * @param bound    the bound that was modified.
       * @param newValue the new value.
       */
      void changed(Axis axis, Bound bound, double newValue);
   }
}
