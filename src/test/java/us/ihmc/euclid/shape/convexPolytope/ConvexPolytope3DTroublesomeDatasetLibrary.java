package us.ihmc.euclid.shape.convexPolytope;

import java.util.Collections;
import java.util.Random;

import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DTroublesomeDataset;
import us.ihmc.euclid.tuple3D.Point3D;

public class ConvexPolytope3DTroublesomeDatasetLibrary
{
   public static class DatasetGJKNullPointerExceptionBug1Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug1Original()
      {
         pointsBeforeIssue.add(new Point3D(0.1727226445760459, 0.3408246844828842, 0.0707692371805856));
         pointsBeforeIssue.add(new Point3D(-0.0010094559634557, 0.1846852819495834, -0.0004136019841320));
         pointsBeforeIssue.add(new Point3D(0.0131019603409140, 0.0679884843470001, 0.0053682349594210));
         pointsBeforeIssue.add(new Point3D(0.0268762363673278, 0.0420820755879367, 0.0110119362210409));
         pointsBeforeIssue.add(new Point3D(0.0382545893376212, 0.0262443294907677, 0.0151346721330367));
         pointsBeforeIssue.add(new Point3D(0.0442214133931064, 0.0188148816283424, 0.0181187342317446));
         pointsBeforeIssue.add(new Point3D(0.0878487223095894, -0.0176017339231975, 0.0359940474533526));
         pointsBeforeIssue.add(new Point3D(0.0317197072764835, 0.0339730913830635, 0.0146770460780876));
         troublesomePoint.set(0.0356799421574458, 0.0305305995258588, 0.0121379970533146);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug1Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug1Simplified()
      {
         // pointsBeforeIssue.add(new Point3D(0.1727226445760459, 0.3408246844828842, 0.0707692371805856));
         // pointsBeforeIssue.add(new Point3D(-0.0010094559634557, 0.1846852819495834, -0.0004136019841320));
         // pointsBeforeIssue.add(new Point3D(0.0131019603409140, 0.0679884843470001, 0.0053682349594210));
         pointsBeforeIssue.add(new Point3D(0.0268762363673278, 0.0420820755879367, 0.0110119362210409));
         pointsBeforeIssue.add(new Point3D(0.0382545893376212, 0.0262443294907677, 0.0151346721330367));
         pointsBeforeIssue.add(new Point3D(0.0442214133931064, 0.0188148816283424, 0.0181187342317446));
         // pointsBeforeIssue.add(new Point3D(0.0878487223095894, -0.0176017339231975, 0.0359940474533526));
         pointsBeforeIssue.add(new Point3D(0.0317197072764835, 0.0339730913830635, 0.0146770460780876));
         troublesomePoint.set(0.0356799421574458, 0.0305305995258588, 0.0121379970533146);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug2Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug2Original()
      {
         pointsBeforeIssue.add(new Point3D(0.1075136584776464, -0.4824945590361311, 0.0090899461159321));
         pointsBeforeIssue.add(new Point3D(0.5607657881997890, 0.4040329322380422, 0.0474110068485305));
         pointsBeforeIssue.add(new Point3D(0.0289073412127943, 0.0078295230557014, -0.0147523787325835));
         pointsBeforeIssue.add(new Point3D(0.0043375074277710, -0.1974921037188990, 0.0003667227899621));
         pointsBeforeIssue.add(new Point3D(0.0139072921615079, -0.0447432688724709, 0.0011758183858351));
         pointsBeforeIssue.add(new Point3D(0.0186428968993924, -0.0235249738131959, 0.0012995156961376));
         pointsBeforeIssue.add(new Point3D(0.0212072625303128, -0.0130016538284213, 0.0026586776633881));
         pointsBeforeIssue.add(new Point3D(0.0341784903074217, 0.0292151862876490, 0.0028896852699534));
         pointsBeforeIssue.add(new Point3D(0.0643309020398054, 0.0996995467973626, 0.0054389780927482));
         pointsBeforeIssue.add(new Point3D(0.0269036070083187, 0.0083216605291816, 0.0078200543380451));
         troublesomePoint.set(0.0325313915792233, 0.0068659303400010, -0.0421477673824605);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug2Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug2Simplified()
      {
         pointsBeforeIssue.add(new Point3D(0.1075136584776464, -0.4824945590361311, 0.0090899461159321));
         pointsBeforeIssue.add(new Point3D(0.5607657881997890, 0.4040329322380422, 0.0474110068485305));
         pointsBeforeIssue.add(new Point3D(0.0289073412127943, 0.0078295230557014, -0.0147523787325835));
         pointsBeforeIssue.add(new Point3D(0.0043375074277710, -0.1974921037188990, 0.0003667227899621));
         pointsBeforeIssue.add(new Point3D(0.0139072921615079, -0.0447432688724709, 0.0011758183858351));
         pointsBeforeIssue.add(new Point3D(0.0186428968993924, -0.0235249738131959, 0.0012995156961376));
         pointsBeforeIssue.add(new Point3D(0.0212072625303128, -0.0130016538284213, 0.0026586776633881));
         pointsBeforeIssue.add(new Point3D(0.0341784903074217, 0.0292151862876490, 0.0028896852699534));
         // pointsBeforeIssue.add(new Point3D(0.0643309020398054, 0.0996995467973626, 0.0054389780927482));
         pointsBeforeIssue.add(new Point3D(0.0269036070083187, 0.0083216605291816, 0.0078200543380451));
         troublesomePoint.set(0.0325313915792233, 0.0068659303400010, -0.0421477673824605);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug3Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug3Original()
      {
         pointsBeforeIssue.add(new Point3D(-0.0037210269862168, 0.0766427555019901, 0.1361514789125481));
         pointsBeforeIssue.add(new Point3D(-0.0098691038563419, -0.0842712352669562, 0.3611081270196839));
         pointsBeforeIssue.add(new Point3D(-0.0013371474796426, 0.1104004526675205, 0.1045058111402412));
         pointsBeforeIssue.add(new Point3D(-0.0036362971658496, 0.0933043645625930, 0.1200567135497745));
         pointsBeforeIssue.add(new Point3D(-0.0024413371416485, 0.1279448469369309, 0.0893279365053837));
         pointsBeforeIssue.add(new Point3D(-0.0064317125888430, 0.1020382766797260, 0.1119023450212495));
         pointsBeforeIssue.add(new Point3D(-0.0012837946563208, 0.1829972596968365, 0.0469737364779878));
         pointsBeforeIssue.add(new Point3D(0.0006330947545868, 0.3027754040393130, -0.0231647841974971));
         pointsBeforeIssue.add(new Point3D(0.0022656588195281, 0.8450024017573801, -0.0828999091198799));
         pointsBeforeIssue.add(new Point3D(-0.0197511024523884, 1.4863939764561370, 0.7226880694514805));
         troublesomePoint.set(0.0153396366110251, 0.1027009848979795, 0.1121607898997810);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug3Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug3Simplified()
      {
         //      pointsBeforeIssue.add(new Point3D(-0.0037210269862168, 0.0766427555019901, 0.1361514789125481));
         //      pointsBeforeIssue.add(new Point3D(-0.0098691038563419, -0.0842712352669562, 0.3611081270196839));
         pointsBeforeIssue.add(new Point3D(-0.0013371474796426, 0.1104004526675205, 0.1045058111402412));
         pointsBeforeIssue.add(new Point3D(-0.0036362971658496, 0.0933043645625930, 0.1200567135497745));
         pointsBeforeIssue.add(new Point3D(-0.0024413371416485, 0.1279448469369309, 0.0893279365053837));
         pointsBeforeIssue.add(new Point3D(-0.0064317125888430, 0.1020382766797260, 0.1119023450212495));
         //      pointsBeforeIssue.add(new Point3D(-0.0012837946563208, 0.1829972596968365, 0.0469737364779878));
         //      pointsBeforeIssue.add(new Point3D(0.0006330947545868, 0.3027754040393130, -0.0231647841974971));
         //      pointsBeforeIssue.add(new Point3D(0.0022656588195281, 0.8450024017573801, -0.0828999091198799));
         pointsBeforeIssue.add(new Point3D(-0.0197511024523884, 1.4863939764561370, 0.7226880694514805));
         troublesomePoint.set(0.0153396366110251, 0.1027009848979795, 0.1121607898997810);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug4Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug4Original()
      {
         pointsBeforeIssue.add(new Point3D(0.0051506551364596, -0.0348819759911019, 0.0691778540151096));
         pointsBeforeIssue.add(new Point3D(0.0422493845451721, 0.0727920949601139, 0.5674466022012327));
         pointsBeforeIssue.add(new Point3D(0.0004104647457426, -0.0451992694194132, 0.0472529766744278));
         pointsBeforeIssue.add(new Point3D(0.0020291613573142, -0.0559465436341543, 0.0250833552689338));
         pointsBeforeIssue.add(new Point3D(-0.0033350301665630, -0.0940280104643934, -0.0447924048275297));
         pointsBeforeIssue.add(new Point3D(-0.0113077830899894, -0.1649473194674913, -0.1518735281452312));
         pointsBeforeIssue.add(new Point3D(-0.0253624237359933, -0.3387902241779525, -0.3406397827449963));
         pointsBeforeIssue.add(new Point3D(-0.0470958837642707, -1.2942558333752370, -0.6325393732333888));
         pointsBeforeIssue.add(new Point3D(0.0046484599886485, -0.0503176108042466, 0.0362947448605222));
         troublesomePoint.set(-0.0068611168376205, -0.0525067429256160, 0.0327580791751562);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug4Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug4Simplified()
      {
         pointsBeforeIssue.add(new Point3D(0.0051506551364596, -0.0348819759911019, 0.0691778540151096));
         // pointsBeforeIssue.add(new Point3D(0.0422493845451721, 0.0727920949601139, 0.5674466022012327));
         pointsBeforeIssue.add(new Point3D(0.0004104647457426, -0.0451992694194132, 0.0472529766744278));
         pointsBeforeIssue.add(new Point3D(0.0020291613573142, -0.0559465436341543, 0.0250833552689338));
         pointsBeforeIssue.add(new Point3D(-0.0033350301665630, -0.0940280104643934, -0.0447924048275297));
         // pointsBeforeIssue.add(new Point3D(-0.0113077830899894, -0.1649473194674913, -0.1518735281452312));
         // pointsBeforeIssue.add(new Point3D(-0.0253624237359933, -0.3387902241779525, -0.3406397827449963));
         // pointsBeforeIssue.add(new Point3D(-0.0470958837642707, -1.2942558333752370, -0.6325393732333888));
         pointsBeforeIssue.add(new Point3D(0.0046484599886485, -0.0503176108042466, 0.0362947448605222));
         troublesomePoint.set(-0.0068611168376205, -0.0525067429256160, 0.0327580791751562);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug5 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug5()
      {
         pointsBeforeIssue.add(new Point3D(0.00987877860785552000, 0.19131147056882847000, 0.03418482283725754500));
         pointsBeforeIssue.add(new Point3D(0.02555035291707655200, 0.11906397969616522000, 0.09162654986018806000));
         pointsBeforeIssue.add(new Point3D(0.03548092653292492000, 0.08553875648964482000, 0.12277926611952411000));
         pointsBeforeIssue.add(new Point3D(0.02835400199038940000, 0.11051604727461928000, 0.09913549801006882000));
         pointsBeforeIssue.add(new Point3D(0.19811056289014100000, 1.53064568455729290000, 0.68554775478046680000));
         troublesomePoint.set(0.03326520528372667000, 0.09790747514458276000, 0.11042444275049243000);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug6 extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug6()
      {
         pointsBeforeIssue.add(new Point3D(-0.16594466242134853000, -0.15167737932767710000, 0.06375940884903786000));
         pointsBeforeIssue.add(new Point3D(-0.15493261840750883000, -0.16646658348557453000, 0.06032647391633983000));
         pointsBeforeIssue.add(new Point3D(-0.18402867588922667000, -0.12629220803748709000, 0.07363271127979126000));
         pointsBeforeIssue.add(new Point3D(-0.20441775948060736000, -0.10131005633356671000, 0.07902696117881491000));
         pointsBeforeIssue.add(new Point3D(-0.28134730614168046000, -0.01222890247491981200, 0.10876756841830115000));
         pointsBeforeIssue.add(new Point3D(0.08114060867902650000, -1.60769973787097800000, -0.03136858435585743000));
         pointsBeforeIssue.add(new Point3D(-0.13409442245051250000, -0.19646058810895028000, 0.05184028405466168000));
         pointsBeforeIssue.add(new Point3D(-0.17583758138846740000, -0.13878848467476557000, 0.06663279438238556000));
         pointsBeforeIssue.add(new Point3D(-1.32350866683800360000, 0.45061221257020545000, 0.51166233452409130000));
         pointsBeforeIssue.add(new Point3D(-0.45317219159206390000, 0.14566075149172053000, 0.17519427511219820000));

         // Could not reproduce the original exception, but manage to highlight a new one by shuffling the pointsBeforeIssue in a given way
         Random random = new Random(478396840718102216L);
         Collections.shuffle(pointsBeforeIssue, random);
         troublesomePoint.set(-0.16885169989877480000, -0.14496535839630130000, 0.07101749538537033000);
         constructionEpsilon = 1.0e-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug7Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug7Original()
      {
         pointsBeforeIssue.add(new Point3D(0.40484035046382340000, 0.66817675595325240000, 0.37565781894372985000));
         pointsBeforeIssue.add(new Point3D(0.19770492108631288000, -0.11722888133146891000, 0.18345354993551521000));
         pointsBeforeIssue.add(new Point3D(0.13177056177929392000, -0.01608972405889597000, 0.12227200619277923000));
         pointsBeforeIssue.add(new Point3D(0.10804284059622116000, 0.04356860347168567400, 0.10025467521792486000));
         pointsBeforeIssue.add(new Point3D(0.10535595585195023000, 0.05183458577843358400, 0.09786307209807943000));
         pointsBeforeIssue.add(new Point3D(0.10419258778046014000, 0.05592435430920772400, 0.09660853251948309000));
         pointsBeforeIssue.add(new Point3D(0.09875689526332210000, 0.07507951247051448000, 0.09163809842020465000));
         pointsBeforeIssue.add(new Point3D(0.09126555383425328000, 0.10746732352304089000, 0.08468676321119539000));
         pointsBeforeIssue.add(new Point3D(0.09891220007971846000, 0.37479443587413260000, 0.09178220823665961000));

         troublesomePoint.set(0.10219794708629126000, 0.06220152556068970000, 0.09504439824552569000);
         constructionEpsilon = 1.0E-4;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug7Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug7Simplified()
      {
         //      pointsBeforeIssue.add(new Point3D( 0.40484035046382340000,  0.66817675595325240000,  0.37565781894372985000 ));
         pointsBeforeIssue.add(new Point3D(0.19770492108631288000, -0.11722888133146891000, 0.18345354993551521000));
         pointsBeforeIssue.add(new Point3D(0.13177056177929392000, -0.01608972405889597000, 0.12227200619277923000));
         pointsBeforeIssue.add(new Point3D(0.10804284059622116000, 0.04356860347168567400, 0.10025467521792486000));
         pointsBeforeIssue.add(new Point3D(0.10535595585195023000, 0.05183458577843358400, 0.09786307209807943000));
         //      pointsBeforeIssue.add(new Point3D( 0.10419258778046014000,  0.05592435430920772400,  0.09660853251948309000 ));
         //      pointsBeforeIssue.add(new Point3D( 0.09875689526332210000,  0.07507951247051448000,  0.09163809842020465000 ));
         //      pointsBeforeIssue.add(new Point3D( 0.09126555383425328000,  0.10746732352304089000,  0.08468676321119539000 ));
         //      pointsBeforeIssue.add(new Point3D( 0.09891220007971846000,  0.37479443587413260000,  0.09178220823665961000 ));

         troublesomePoint.set(0.10219794708629126000, 0.06220152556068970000, 0.09504439824552569000);
         constructionEpsilon = 1.0E-4;
      }
   }

   public static class DatasetEPAFaceNormalIntegrityBug8Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetEPAFaceNormalIntegrityBug8Original()
      {
         pointsBeforeIssue.add(new Point3D(0.25923025651672880000, -1.32459442213312100000, 0.33731064563758340000));
         pointsBeforeIssue.add(new Point3D(-0.58211725366830260000, -0.18145508297583146000, -0.75745150010666570000));
         pointsBeforeIssue.add(new Point3D(-0.29186699291394436000, -0.26463480549211205000, -0.37977759673184330000));
         pointsBeforeIssue.add(new Point3D(-0.15759191168890652000, -0.36628560982904357000, -0.20505873887300530000));
         pointsBeforeIssue.add(new Point3D(-0.14713791925305353000, -0.37644945730058765000, -0.19100593449992065000));
         pointsBeforeIssue.add(new Point3D(-0.14153660446287397000, -0.38162347173657220000, -0.18434170307648423000));
         pointsBeforeIssue.add(new Point3D(-0.12585187210813410000, -0.39752753793187856000, -0.16375857049215160000));
         pointsBeforeIssue.add(new Point3D(-0.09496404628807320000, -0.43099089641061805000, -0.12356730343211364000));
         pointsBeforeIssue.add(new Point3D(-0.03597410587601624000, -0.50432786022543690000, -0.04680953929654707600));

         troublesomePoint.set(-0.13693975331059360000, -0.38652881789871840000, -0.17763839169767880000);
         constructionEpsilon = 5.0E-4;
      }
   }

   public static class DatasetEPAFaceNormalIntegrityBug8Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetEPAFaceNormalIntegrityBug8Simplified()
      {
         //      pointsBeforeIssue.add(new Point3D( 0.25923025651672880000, -1.32459442213312100000,  0.33731064563758340000 ));
         pointsBeforeIssue.add(new Point3D(-0.58211725366830260000, -0.18145508297583146000, -0.75745150010666570000));
         //      pointsBeforeIssue.add(new Point3D(-0.29186699291394436000, -0.26463480549211205000, -0.37977759673184330000 ));
         pointsBeforeIssue.add(new Point3D(-0.15759191168890652000, -0.36628560982904357000, -0.20505873887300530000));
         pointsBeforeIssue.add(new Point3D(-0.14713791925305353000, -0.37644945730058765000, -0.19100593449992065000));
         pointsBeforeIssue.add(new Point3D(-0.14153660446287397000, -0.38162347173657220000, -0.18434170307648423000));
         //      pointsBeforeIssue.add(new Point3D(-0.12585187210813410000, -0.39752753793187856000, -0.16375857049215160000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.09496404628807320000, -0.43099089641061805000, -0.12356730343211364000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.03597410587601624000, -0.50432786022543690000, -0.04680953929654707600 ));

         troublesomePoint.set(-0.13693975331059360000, -0.38652881789871840000, -0.17763839169767880000);
         constructionEpsilon = 5.0E-4;
      }
   }

   public static class DatasetEPAFaceNormalIntegrityBug9Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetEPAFaceNormalIntegrityBug9Original()
      {
         pointsBeforeIssue.add(new Point3D(0.25923025651672880000, -1.32459442213312100000, 0.33731064563758340000));
         pointsBeforeIssue.add(new Point3D(-0.58211725366830260000, -0.18145508297583146000, -0.75745150010666570000));
         pointsBeforeIssue.add(new Point3D(-0.29186699291394436000, -0.26463480549211205000, -0.37977759673184330000));
         pointsBeforeIssue.add(new Point3D(-0.15759191168890652000, -0.36628560982904357000, -0.20505873887300530000));
         pointsBeforeIssue.add(new Point3D(-0.14713791925305353000, -0.37644945730058765000, -0.19100593449992065000));
         pointsBeforeIssue.add(new Point3D(-0.14153660446287397000, -0.38162347173657220000, -0.18434170307648423000));
         pointsBeforeIssue.add(new Point3D(-0.12585187210813410000, -0.39752753793187856000, -0.16375857049215160000));
         pointsBeforeIssue.add(new Point3D(-0.09496404628807320000, -0.43099089641061805000, -0.12356730343211364000));
         pointsBeforeIssue.add(new Point3D(-0.03597410587601624000, -0.50432786022543690000, -0.04680953929654707600));

         troublesomePoint.set(-0.13693975331059360000, -0.38652881789871840000, -0.17763839169767880000);
         constructionEpsilon = 5.0E-4;
      }
   }

   public static class DatasetEPAFaceNormalIntegrityBug9Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetEPAFaceNormalIntegrityBug9Simplified()
      {
         //      pointsBeforeIssue.add(new Point3D( 0.25923025651672880000, -1.32459442213312100000,  0.33731064563758340000 ));
         pointsBeforeIssue.add(new Point3D(-0.58211725366830260000, -0.18145508297583146000, -0.75745150010666570000));
         pointsBeforeIssue.add(new Point3D(-0.29186699291394436000, -0.26463480549211205000, -0.37977759673184330000));
         pointsBeforeIssue.add(new Point3D(-0.15759191168890652000, -0.36628560982904357000, -0.20505873887300530000));
         pointsBeforeIssue.add(new Point3D(-0.14713791925305353000, -0.37644945730058765000, -0.19100593449992065000));
         pointsBeforeIssue.add(new Point3D(-0.14153660446287397000, -0.38162347173657220000, -0.18434170307648423000));
         //      pointsBeforeIssue.add(new Point3D(-0.12585187210813410000, -0.39752753793187856000, -0.16375857049215160000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.09496404628807320000, -0.43099089641061805000, -0.12356730343211364000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.03597410587601624000, -0.50432786022543690000, -0.04680953929654707600 ));

         troublesomePoint.set(-0.13693975331059360000, -0.38652881789871840000, -0.17763839169767880000);
         constructionEpsilon = 5.0E-4;
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug10Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug10Original()
      {
         pointsBeforeIssue.add(new Point3D(-0.17317378540777761000, -0.05775495286524568000, 0.14175166115938010000));
         pointsBeforeIssue.add(new Point3D(-0.01897404558870108300, 0.00308339916520594670, 0.01553123340681406000));
         pointsBeforeIssue.add(new Point3D(-0.00274787469423487530, 0.01193080363919030200, 0.00214472895451733870));
         pointsBeforeIssue.add(new Point3D(-0.35181526244719350000, 2.19178801525574850000, 0.28797890948495250000));
         pointsBeforeIssue.add(new Point3D(0.52312335359372390000, 1.17120574040232170000, -0.42820340381521560000));
         pointsBeforeIssue.add(new Point3D(0.35783508614818305000, 0.38370415108791240000, -0.29290644518264797000));
         pointsBeforeIssue.add(new Point3D(0.12404154368939402000, 0.09927405350457708000, -0.10153439118595142000));
         pointsBeforeIssue.add(new Point3D(0.05424289486816125000, 0.04693494504728657000, -0.04440060275503699000));
         pointsBeforeIssue.add(new Point3D(0.01802380347325072000, 0.02392344071554553000, -0.01475341131840446400));
         pointsBeforeIssue.add(new Point3D(-0.00038260858106387020, 0.01322913616138221600, 0.00031318545046343793));
         pointsBeforeIssue.add(new Point3D(-0.00965583175989204200, 0.00808735606657740000, 0.00790374855115949500));
         pointsBeforeIssue.add(new Point3D(-0.00501274681955910000, 0.01064106008930820700, 0.00410473368994557200));
         pointsBeforeIssue.add(new Point3D(-0.00129912505345441960, 0.01133059642016243100, 0.00613679617411944100));
         pointsBeforeIssue.add(new Point3D(-0.00280702931313747700, 0.01130231935193704000, 0.00437573895350351600));
         pointsBeforeIssue.add(new Point3D(-0.00386839007299566440, 0.01128503414358750200, 0.00313791547173392100));

         troublesomePoint.set(-0.00276525936788729600, 0.01161966868636366000, 0.00326143254637401000);
         constructionEpsilon = 1.0E-7;
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug10Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug10Simplified()
      {
         //      pointsBeforeIssue.add(new Point3D(-0.17317378540777761000, -0.05775495286524568000, 0.14175166115938010000));
         //      pointsBeforeIssue.add(new Point3D(-0.01897404558870108300, 0.00308339916520594670, 0.01553123340681406000));
         pointsBeforeIssue.add(new Point3D(-0.00274787469423487530, 0.01193080363919030200, 0.00214472895451733870));
         //      pointsBeforeIssue.add(new Point3D(-0.35181526244719350000, 2.19178801525574850000, 0.28797890948495250000));
         //      pointsBeforeIssue.add(new Point3D(0.52312335359372390000, 1.17120574040232170000, -0.42820340381521560000));
         //      pointsBeforeIssue.add(new Point3D(0.35783508614818305000, 0.38370415108791240000, -0.29290644518264797000));
         //      pointsBeforeIssue.add(new Point3D(0.12404154368939402000, 0.09927405350457708000, -0.10153439118595142000));
         //      pointsBeforeIssue.add(new Point3D(0.05424289486816125000, 0.04693494504728657000, -0.04440060275503699000));
         //      pointsBeforeIssue.add(new Point3D(0.01802380347325072000, 0.02392344071554553000, -0.01475341131840446400));
         //      pointsBeforeIssue.add(new Point3D(-0.00038260858106387020, 0.01322913616138221600, 0.00031318545046343793));
         //      pointsBeforeIssue.add(new Point3D(-0.00965583175989204200, 0.00808735606657740000, 0.00790374855115949500));
         pointsBeforeIssue.add(new Point3D(-0.00501274681955910000, 0.01064106008930820700, 0.00410473368994557200));
         //      pointsBeforeIssue.add(new Point3D(-0.00129912505345441960, 0.01133059642016243100, 0.00613679617411944100));
         pointsBeforeIssue.add(new Point3D(-0.00280702931313747700, 0.01130231935193704000, 0.00437573895350351600));
         pointsBeforeIssue.add(new Point3D(-0.00386839007299566440, 0.01128503414358750200, 0.00313791547173392100));

         troublesomePoint.set(-0.00276525936788729600, 0.01161966868636366000, 0.00326143254637401000);
         constructionEpsilon = 1.0E-7;
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug11Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug11Original()
      {
         pointsBeforeIssue.add(new Point3D(-0.41935432254164250000, -0.49779783391735880000, 0.51288709338555660000));
         pointsBeforeIssue.add(new Point3D(-0.93298416837861000000, 1.29815954722883200000, 1.14107691890293680000));
         pointsBeforeIssue.add(new Point3D(-0.30925383873489704000, 0.66208031898425670000, 0.37822980220101690000));
         pointsBeforeIssue.add(new Point3D(-0.25512404662567070000, 0.06730540401802787000, 0.31202690348710493000));
         pointsBeforeIssue.add(new Point3D(-0.25507813765207943000, 0.06013191657288902000, 0.31339072703416770000));
         pointsBeforeIssue.add(new Point3D(-0.25531309357128740000, 0.06263846976145082000, 0.31272772115361480000));
         pointsBeforeIssue.add(new Point3D(-0.25641070430508184000, 0.05800263215782730500, 0.31270661888213390000));
         pointsBeforeIssue.add(new Point3D(-0.25641896081503250000, 0.05332514660554210600, 0.31361063528392374000));
         Collections.shuffle(pointsBeforeIssue, new Random(7106315192527041498L));

         troublesomePoint.set(-0.25784327149254240000, 0.05771251668497923000, 0.31159424166064453000);
         constructionEpsilon = 1.0E-3;
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug11Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug11Simplified()
      {
         pointsBeforeIssue.add(new Point3D(-0.41935432254164250000, -0.49779783391735880000, 0.51288709338555660000));
         pointsBeforeIssue.add(new Point3D(-0.93298416837861000000, 1.29815954722883200000, 1.14107691890293680000));
         pointsBeforeIssue.add(new Point3D(-0.30925383873489704000, 0.66208031898425670000, 0.37822980220101690000));
         //      pointsBeforeIssue.add(new Point3D(-0.25512404662567070000,  0.06730540401802787000,  0.31202690348710493000 ));
         pointsBeforeIssue.add(new Point3D(-0.25507813765207943000, 0.06013191657288902000, 0.31339072703416770000));
         pointsBeforeIssue.add(new Point3D(-0.25531309357128740000, 0.06263846976145082000, 0.31272772115361480000));
         pointsBeforeIssue.add(new Point3D(-0.25641070430508184000, 0.05800263215782730500, 0.31270661888213390000));
         pointsBeforeIssue.add(new Point3D(-0.25641896081503250000, 0.05332514660554210600, 0.31361063528392374000));
         Collections.shuffle(pointsBeforeIssue, new Random(7106315192527041498L));

         troublesomePoint.set(-0.25784327149254240000, 0.05771251668497923000, 0.31159424166064453000);
         constructionEpsilon = 1.0E-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug12Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug12Original()
      {
         pointsBeforeIssue.add(new Point3D(-1.06285928606209360000, 0.84863501402184480000, -0.36386991230254034000));
         pointsBeforeIssue.add(new Point3D(-0.24794687685778950000, -0.88150811138554650000, -0.08488462162493815000));
         pointsBeforeIssue.add(new Point3D(-0.14860460603249515000, 0.25024717144708375000, -0.05087479186933352400));
         pointsBeforeIssue.add(new Point3D(-0.04593802791056822400, -0.33056946184021730000, -0.01572688539900857200));
         pointsBeforeIssue.add(new Point3D(-0.05894221041959524000, -0.03259192667445954000, -0.02017886771799948200));
         pointsBeforeIssue.add(new Point3D(-0.09448585378890406000, 0.11211693061937889000, -0.03234723522001137000));
         pointsBeforeIssue.add(new Point3D(-0.07434099339443390000, 0.04041368632546394000, -0.02545064158692189600));
         pointsBeforeIssue.add(new Point3D(-0.06604318267016518000, 0.00405189569758113160, -0.02260988580670941800));
         pointsBeforeIssue.add(new Point3D(-0.06234252506146254000, -0.01423750400223156000, -0.02134296554921921700));
         pointsBeforeIssue.add(new Point3D(-0.06460601902846741000, -0.00286855106762995400, -0.02211787279677335200));
         pointsBeforeIssue.add(new Point3D(-0.06167537739882600000, -0.01770921927976720800, -0.02111456740931594300));
         pointsBeforeIssue.add(new Point3D(-0.06762831802521285000, 0.01144471242318934000, -0.02315255872024968300));
         pointsBeforeIssue.add(new Point3D(-0.06306822611543828000, -0.01052577856589109700, -0.02159139726896253000));
         pointsBeforeIssue.add(new Point3D(-0.06095078718494506600, -0.02154749334084593600, -0.02086667479886617000));
         pointsBeforeIssue.add(new Point3D(-0.06721513866931716000, 0.00953728141767107300, -0.02300866411810875700));
         pointsBeforeIssue.add(new Point3D(-0.06287435077799386000, -0.01148375396021106300, -0.02154246977939222600));
         pointsBeforeIssue.add(new Point3D(-0.06094046529089814000, -0.02202728685290667000, -0.02060449498172178800));
         pointsBeforeIssue.add(new Point3D(-0.06604214004639730000, 0.00928429102395866800, -0.02627429468814690400));
         pointsBeforeIssue.add(new Point3D(-0.06782601695094026000, -0.00492098617148972700, -0.01152854836332983800));
         pointsBeforeIssue.add(new Point3D(-0.06208347428735250000, -0.00677686010372791100, -0.02694477655902294300));

         troublesomePoint.set(-0.06208347428735250000, -0.00677686010372791100, -0.02694477655902294300);
         constructionEpsilon = 1.0E-3;
      }
   }

   public static class DatasetGJKNullPointerExceptionBug12Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKNullPointerExceptionBug12Simplified()
      {
         pointsBeforeIssue.add(new Point3D(-1.06285928606209360000, 0.84863501402184480000, -0.36386991230254034000));
         //      pointsBeforeIssue.add(new Point3D(-0.24794687685778950000, -0.88150811138554650000, -0.08488462162493815000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.14860460603249515000,  0.25024717144708375000, -0.05087479186933352400 ));
         //      pointsBeforeIssue.add(new Point3D(-0.04593802791056822400, -0.33056946184021730000, -0.01572688539900857200 ));
         pointsBeforeIssue.add(new Point3D(-0.05894221041959524000, -0.03259192667445954000, -0.02017886771799948200));
         //      pointsBeforeIssue.add(new Point3D(-0.09448585378890406000,  0.11211693061937889000, -0.03234723522001137000 ));
         pointsBeforeIssue.add(new Point3D(-0.07434099339443390000, 0.04041368632546394000, -0.02545064158692189600));
         //      pointsBeforeIssue.add(new Point3D(-0.06604318267016518000,  0.00405189569758113160, -0.02260988580670941800 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06234252506146254000, -0.01423750400223156000, -0.02134296554921921700 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06460601902846741000, -0.00286855106762995400, -0.02211787279677335200 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06167537739882600000, -0.01770921927976720800, -0.02111456740931594300 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06762831802521285000,  0.01144471242318934000, -0.02315255872024968300 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06306822611543828000, -0.01052577856589109700, -0.02159139726896253000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06095078718494506600, -0.02154749334084593600, -0.02086667479886617000 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06721513866931716000,  0.00953728141767107300, -0.02300866411810875700 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06287435077799386000, -0.01148375396021106300, -0.02154246977939222600 ));
         pointsBeforeIssue.add(new Point3D(-0.06094046529089814000, -0.02202728685290667000, -0.02060449498172178800));
         pointsBeforeIssue.add(new Point3D(-0.06604214004639730000, 0.00928429102395866800, -0.02627429468814690400));
         //      pointsBeforeIssue.add(new Point3D(-0.06782601695094026000, -0.00492098617148972700, -0.01152854836332983800 ));
         //      pointsBeforeIssue.add(new Point3D(-0.06208347428735250000, -0.00677686010372791100, -0.02694477655902294300 ));

         troublesomePoint.set(-0.06208347428735250000, -0.00677686010372791100, -0.02694477655902294300);
         constructionEpsilon = 1.0E-3;
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug13Original extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug13Original()
      {
         pointsBeforeIssue.add(new Point3D(-1.16485579659865150000, 1.50478742011243050000, 0.92998233563902600000));
         pointsBeforeIssue.add(new Point3D(-0.49671459562836695000, -0.57321041192247600000, 0.39656050228474937000));
         pointsBeforeIssue.add(new Point3D(-0.28684916277252004000, 0.75215556219304490000, 0.22901088284939053000));
         pointsBeforeIssue.add(new Point3D(-0.23434852727844335000, 0.04865437022396490500, 0.18709611214396826000));
         pointsBeforeIssue.add(new Point3D(-0.22769504113460493000, 0.09534067425821080000, 0.18178418889796938000));
         pointsBeforeIssue.add(new Point3D(-0.23084361780005236000, 0.07195594672506622000, 0.18429790835562376000));
         pointsBeforeIssue.add(new Point3D(-0.23255161132903990000, 0.06029420820407039000, 0.18566151389030200000));
         pointsBeforeIssue.add(new Point3D(-0.23343896451436674000, 0.05447148220883074600, 0.18636994732063494000));
         pointsBeforeIssue.add(new Point3D(-0.23084251150272717000, 0.07196370955534642000, 0.18429702512443302000));
         pointsBeforeIssue.add(new Point3D(-0.23255102857838360000, 0.06029808252032909000, 0.18566104864145722000));
         pointsBeforeIssue.add(new Point3D(-0.23343866574888150000, 0.05447341752291190400, 0.18636970879614645000));
         pointsBeforeIssue.add(new Point3D(-0.23084361798631325000, 0.07195594541800332000, 0.18429790850435868000));
         pointsBeforeIssue.add(new Point3D(-0.23255161142718340000, 0.06029420755173304000, 0.18566151396861086000));
         pointsBeforeIssue.add(new Point3D(-0.23343896456459690000, 0.05447148188295081000, 0.18636994736089696000));
         pointsBeforeIssue.add(new Point3D(-0.23084251131672584000, 0.07196371086271980000, 0.18429702497529600000));
         pointsBeforeIssue.add(new Point3D(-0.23255102847977416000, 0.06029808317281127000, 0.18566104856369010000));
         pointsBeforeIssue.add(new Point3D(-0.23343866570020700000, 0.05447341784881787000, 0.18636970875392822000));
         pointsBeforeIssue.add(new Point3D(-0.23084361816609666000, 0.07195594411052708000, 0.18429790866132790000));
         pointsBeforeIssue.add(new Point3D(-0.23255161153516546000, 0.06029420689919168000, 0.18566151403466002000));
         pointsBeforeIssue.add(new Point3D(-0.23343896458049573000, 0.05447148155701498000, 0.18636994744417840000));
         pointsBeforeIssue.add(new Point3D(-0.23084251126809020000, 0.07196371217031738000, 0.18429702465403558000));
         pointsBeforeIssue.add(new Point3D(-0.23255102817507890000, 0.06029808382542023400, 0.18566104874401880000));
         pointsBeforeIssue.add(new Point3D(-0.23343866637276056000, 0.05447341817483153400, 0.18636970780829720000));
         pointsBeforeIssue.add(new Point3D(-0.23084361546016796000, 0.07195594280261897000, 0.18429791243294170000));
         pointsBeforeIssue.add(new Point3D(-0.23255161597315000000, 0.06029420624642717000, 0.18566150867720305000));
         pointsBeforeIssue.add(new Point3D(-0.23343894943555432000, 0.05447148123095147000, 0.18636996651731620000));
         pointsBeforeIssue.add(new Point3D(-0.23084257187954450000, 0.07196371347842823000, 0.18429694835241683000));
         pointsBeforeIssue.add(new Point3D(-0.23255093689286900000, 0.06029808447907076000, 0.18566116287855428000));
         pointsBeforeIssue.add(new Point3D(-0.23343898546925080000, 0.05447341850844744600, 0.18636930801633494000));
         pointsBeforeIssue.add(new Point3D(-0.23084233870854210000, 0.07195594146460049000, 0.18429951203091433000));
         pointsBeforeIssue.add(new Point3D(-0.23255353201902773000, 0.06029420592756607000, 0.18565910882543690000));
         pointsBeforeIssue.add(new Point3D(-0.23343224039050580000, 0.05447148421488279000, 0.18637836911135863000));
         pointsBeforeIssue.add(new Point3D(-0.23086941605680810000, 0.07196370167909760000, 0.18426332918815225000));
         pointsBeforeIssue.add(new Point3D(-0.23251065898995815000, 0.06029823265925538000, 0.18571157034205898000));
         pointsBeforeIssue.add(new Point3D(-0.23357967126484824000, 0.05447487912998916000, 0.18619266304844595000));
         pointsBeforeIssue.add(new Point3D(-0.23027969794173497000, 0.07195015744675376000, 0.18500649211601283000));
         pointsBeforeIssue.add(new Point3D(-0.23190458293936178000, 0.06320641868777910000, 0.18557954893731465000));
         pointsBeforeIssue.add(new Point3D(-0.23378371205299409000, 0.05595052063483186000, 0.18547193991612554000));
         pointsBeforeIssue.add(new Point3D(-0.23118909226302153000, 0.05911705963689206600, 0.18773635485865492000));
         pointsBeforeIssue.add(new Point3D(-0.23349476843202010000, 0.05976000785206925000, 0.18464643281306692000));

         troublesomePoint.set(-0.23349476843202010000, 0.05976000785206925000, 0.18464643281306692000);
         constructionEpsilon = 1.0E-3;
      }
   }

   public static class DatasetGJKFaceNormalIntegrityBug13Simplified extends ConvexPolytope3DTroublesomeDataset
   {
      public DatasetGJKFaceNormalIntegrityBug13Simplified()
      {
         pointsBeforeIssue.add(new Point3D(-1.16485579659865150000, 1.50478742011243050000, 0.92998233563902600000));
         pointsBeforeIssue.add(new Point3D(-0.49671459562836695000, -0.57321041192247600000, 0.39656050228474937000));
         pointsBeforeIssue.add(new Point3D(-0.28684916277252004000, 0.75215556219304490000, 0.22901088284939053000));
         pointsBeforeIssue.add(new Point3D(-0.23434852727844335000, 0.04865437022396490500, 0.18709611214396826000));
         pointsBeforeIssue.add(new Point3D(-0.22769504113460493000, 0.09534067425821080000, 0.18178418889796938000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084361780005236000, 0.07195594672506622000, 0.18429790835562376000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255161132903990000, 0.06029420820407039000, 0.18566151389030200000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343896451436674000, 0.05447148220883074600, 0.18636994732063494000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084251150272717000, 0.07196370955534642000, 0.18429702512443302000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255102857838360000, 0.06029808252032909000, 0.18566104864145722000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343866574888150000, 0.05447341752291190400, 0.18636970879614645000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084361798631325000, 0.07195594541800332000, 0.18429790850435868000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255161142718340000, 0.06029420755173304000, 0.18566151396861086000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343896456459690000, 0.05447148188295081000, 0.18636994736089696000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084251131672584000, 0.07196371086271980000, 0.18429702497529600000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255102847977416000, 0.06029808317281127000, 0.18566104856369010000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343866570020700000, 0.05447341784881787000, 0.18636970875392822000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084361816609666000, 0.07195594411052708000, 0.18429790866132790000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255161153516546000, 0.06029420689919168000, 0.18566151403466002000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343896458049573000, 0.05447148155701498000, 0.18636994744417840000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084251126809020000, 0.07196371217031738000, 0.18429702465403558000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255102817507890000, 0.06029808382542023400, 0.18566104874401880000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343866637276056000, 0.05447341817483153400, 0.18636970780829720000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084361546016796000, 0.07195594280261897000, 0.18429791243294170000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255161597315000000, 0.06029420624642717000, 0.18566150867720305000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343894943555432000, 0.05447148123095147000, 0.18636996651731620000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084257187954450000, 0.07196371347842823000, 0.18429694835241683000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255093689286900000, 0.06029808447907076000, 0.18566116287855428000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343898546925080000, 0.05447341850844744600, 0.18636930801633494000));
         //      pointsBeforeIssue.add(new Point3D(-0.23084233870854210000, 0.07195594146460049000, 0.18429951203091433000));
         //      pointsBeforeIssue.add(new Point3D(-0.23255353201902773000, 0.06029420592756607000, 0.18565910882543690000));
         //      pointsBeforeIssue.add(new Point3D(-0.23343224039050580000, 0.05447148421488279000, 0.18637836911135863000));
         //      pointsBeforeIssue.add(new Point3D(-0.23086941605680810000, 0.07196370167909760000, 0.18426332918815225000));
         //      pointsBeforeIssue.add(new Point3D(-0.23251065898995815000, 0.06029823265925538000, 0.18571157034205898000));
         //      pointsBeforeIssue.add(new Point3D(-0.23357967126484824000, 0.05447487912998916000, 0.18619266304844595000));
         //      pointsBeforeIssue.add(new Point3D(-0.23027969794173497000, 0.07195015744675376000, 0.18500649211601283000));
         pointsBeforeIssue.add(new Point3D(-0.23190458293936178000, 0.06320641868777910000, 0.18557954893731465000));
         pointsBeforeIssue.add(new Point3D(-0.23378371205299409000, 0.05595052063483186000, 0.18547193991612554000));
         pointsBeforeIssue.add(new Point3D(-0.23118909226302153000, 0.05911705963689206600, 0.18773635485865492000));
         //      pointsBeforeIssue.add(new Point3D(-0.23349476843202010000, 0.05976000785206925000, 0.18464643281306692000));

         troublesomePoint.set(-0.23349476843202010000, 0.05976000785206925000, 0.18464643281306692000);
         constructionEpsilon = 1.0E-3;
      }
   }
}
