#ifndef COLORMAP_H__
#define COLORMAP_H__
 
class ColorMap
{
public:
	ColorMap();
	~ColorMap();

	virtual double getR(double val)=0;
	virtual double getG(double val)=0;
	virtual double getB(double val)=0;
};

// --- red - yellow - white ---

class DensityColorMap : public ColorMap{
public:
	DensityColorMap(){};
	~DensityColorMap(){};

	double getR(double val){
		return 1.0;
	};
	double getG(double val){
		if(val <= 0.5){
			return 1.0;
		}
		return 1.0-(val-0.5)/0.5;
	};
	double getB(double val){
		if(val > 0.5){
			return 0;
		}
		return 1.0 -val/0.5;
	};		
};


// --- white - red - yellow ---
/*
class DensityColorMap : public ColorMap{
public:
    DensityColorMap(){};
    ~DensityColorMap(){};

    double getR(double val){
        return 1.0;     
    };
    double getG(double val){
        if(val <= 0.5){
          return 1.0 -val/0.5;            
        }
        return (val-0.5)/0.5;        
    };
    double getB(double val){
        if(val <= 0.5){
          return 0;
        }
        return (val-0.5)/0.5;
    };      
};
*/

// --- red - white - blue
/*
class DensityColorMap : public ColorMap{
public:
    DensityColorMap(){};
    ~DensityColorMap(){};

    double getR(double val){
        if(val >= 0.5){
          return 1.0;     
        }
        return val/0.5;        
    };
    double getG(double val){
        if(val <= 0.5){
          //return 1.0-(0.5-val)/0.5;            
          return val/0.5;  
        }
        return 1.0-(val-0.5)/0.5;        
    };
    double getB(double val){
        if(val >= 0.5){
          return 0;
        }
        return 1.0-(val)/0.5;
    };      
};
*/
class HeatColorMap : public ColorMap{
public:
	HeatColorMap(){};
	~HeatColorMap(){};	

	double getR(double v){
    double ret = 0.0;
    if(v < 0.5){
        ret = 0.0;
    }else if(v > 0.5 && v < 0.745){
        ret = (v-0.5)*4;
    }else if(v > 0.745){
        ret = 1.0;
    }
    return ret;
  }

  double getG(double v){
    double ret = 0.0;
    if(v >= 0.25 && v <= 0.745){
        ret = 1.0;
    }else if(v < 0.25){
        ret = v * 4;
    }else{
        ret = 1.0-(v-0.745)*4;
    }
    return ret;
  }

  double getB(double v){
    double ret = 0.0;
    if(v <= 0.25){
        ret = 1.0;
    }else if(v > 0.25 && v < 0.5){
        ret = 1.0-(v-0.25)*4;
    }else if(v >= 0.5){
        ret = 0;
    }
    return ret;
  }	
};

/**
 * @brief Binary color map
 * @details if val > 0, return 1.0, 1.0, 1.0
 *          else, return 0.0, 0.0, 0.0
 */
class BinaryColorMap : public ColorMap{
  public:
  BinaryColorMap(){};
  ~BinaryColorMap(){};

  double getR(double val){
    if(val > 0.0){
      return 1.0;
    }
    return 0.0;
  };

  double getG(double val){
    if(val > 0.0){
      return 1.0;
    }
    return 0.0;
  };

  double getB(double val){
    if(val > 0.0){
      return 1.0;
    }
    return 0.0;
  };
};

class DivergingColorMap : public ColorMap{
  DivergingColorMap(){};
  ~DivergingColorMap(){};

  double getR(double val){
    return val;
  };

  double getG(double val){
    return val;
  };

  double getB(double val){
    return val;
  };

};
 
#endif /* end of include guard: COLORMAP_H__ */
