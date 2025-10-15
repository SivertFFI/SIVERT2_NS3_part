/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 CTTC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Nicola Baldo <nbaldo@cttc.es>
 */
// SPDX-License-Identifier: GPL-2.0-only
// SPDX-FileCopyrightText: 2025 Aleksei Fedorov <aleksei.fedorov@eit.lth.se>
// SPDX-FileCopyrightText: 2025 Nikita Lyamin <nikita.lyamin@volvocars.com>

#include <ns3/mobility-model.h>
#include <ns3/sivert-spectrum-propagation-loss.h>
#include <ns3/uinteger.h>
#include <ns3/spectrum-signal-parameters.h>


namespace ns3 {

NS_LOG_COMPONENT_DEFINE("SivertSpectrumPropagationLossModel");
NS_OBJECT_ENSURE_REGISTERED (SivertSpectrumPropagationLossModel);


SivertSpectrumPropagationLossModel::~SivertSpectrumPropagationLossModel ()
{
}

SivertSpectrumPropagationLossModel::SivertSpectrumPropagationLossModel()
{
}

void SivertSpectrumPropagationLossModel::DoInitialize()
{
  double init_gain = 1e-25;
  std::cout<< "Num of Sub-channels: " << m_number_of_subchannels << std::endl;
  for (int i = 0; i < 100 ; ++i) {
      for (int j = 0; j < 100; ++j) {
          for (int k = 0; k < m_number_of_subchannels; ++k)
            {
              m_Channel_from_Unity3D[i][j].push_back(1.0);
            }
        }
    }
}


TypeId
SivertSpectrumPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SivertSpectrumPropagationLossModel")
    .SetParent<SpectrumPropagationLossModel> ()
    .SetGroupName ("Spectrum")
    .AddConstructor<SivertSpectrumPropagationLossModel> ()
    .AddAttribute("NumFreq",
                  "Number of sub-channels",
                  UintegerValue (30),
                  MakeUintegerAccessor(&SivertSpectrumPropagationLossModel::m_number_of_subchannels),
                  MakeUintegerChecker<uint16_t> (1, 200))
  ;
  return tid;
}



Ptr<SpectrumValue>
SivertSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity (Ptr<const SpectrumSignalParameters> params,
                                                                 Ptr<const MobilityModel> a,
                                                                 Ptr<const MobilityModel> b) const
{

  Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue> (params->psd);
  Values::iterator vit = rxPsd->ValuesBegin ();
  Bands::const_iterator fit = rxPsd->ConstBandsBegin ();
  NS_LOG_DEBUG("DoCalcRxPowerSpectralDensity - non-SIVERT model");

  NS_ASSERT (a);
  NS_ASSERT (b);


  Values::iterator first_RB_index = rxPsd->ValuesBegin ();

  double d = a->GetDistanceFrom (b);

  while (vit != rxPsd->ValuesEnd ())
    {
      BandInfo fr = *fit;
      int RBnumber = vit - first_RB_index;
      NS_ASSERT (fit != rxPsd->ConstBandsEnd ());
    //  NS_LOG_DEBUG("RB number: " << RBnumber << std::endl);
    //  NS_LOG_DEBUG("RB frequency: " << fr.fc << std::endl);
    //  NS_LOG_DEBUG("Sivert Spectrum PSD Value before: " << *vit << std::endl);
      *vit /= CalculateLoss (fit->fc, d); // Prx = Ptx / loss
      // NS_LOG_DEBUG("Sivert Spectrum PSD Value after: " << *vit << std::endl);
      ++vit;
      ++fit;
    }
  return rxPsd;
}


double
SivertSpectrumPropagationLossModel::CalculateLoss (double f, double d) const
{
  NS_ASSERT (d >= 0);

  if (d == 0)
    {
      return 1;
    }

  NS_ASSERT (f > 0);
  double loss_sqrt = (4 * M_PI * f * d) / 3e8;
  double loss = loss_sqrt * loss_sqrt;

  if (loss < 1)
    {
      loss = 1;
    }
  return loss;
}


Ptr<SpectrumValue>
SivertSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensitySIVERT (Ptr<const SpectrumSignalParameters> txPsd,
                                                                  int TxID,
                                                                  int RxID) const
{


  double totalRxPowerW = 0.0;
  Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue> (txPsd->psd);
  Values::iterator vit = rxPsd->ValuesBegin ();
  Bands::const_iterator fit = rxPsd->ConstBandsBegin ();
  BandInfo fr = *fit;
  double bwHz = fr.fh - fr.fl;
  std::vector<double> psd_vector = m_Channel_from_Unity3D[TxID][RxID];
  NS_LOG_DEBUG(this << " Link: RxID: "<< RxID << " TxID: "<< TxID << std::endl);
  Values::iterator first_RB_index = rxPsd->ValuesBegin ();
  NS_LOG_DEBUG("m_Channel_from_Unity3 vector size: "<< psd_vector.size() << " PSD size for the sub-band used by Tx-Rx (# sub-carrieris): " << rxPsd->ValuesEnd () - rxPsd->ValuesBegin ()  <<  std::endl);
  

//  std::cout<< "rxPSD size: "<< rxPsd->ValuesBegin() - rxPsd->ValuesEnd() << std::endl;

  while (vit != rxPsd->ValuesEnd ())
    {
      if (TxID!=RxID){
        BandInfo fr = *fit;
        int RBnumber = vit - first_RB_index;
        // NS_ASSERT (fit != rxPsd->ConstBandsEnd ());
        NS_LOG_INFO("RB number: " << RBnumber << std::endl);
        NS_LOG_INFO("RB frequency: " << fr.fc << std::endl);
        NS_LOG_INFO("Sivert Spectrum PSD Value before: " << *vit << std::endl);
        NS_LOG_INFO("PSD gain received from Unity3D: " << psd_vector[RBnumber] << std::endl);
        *vit *= psd_vector[RBnumber]; // Prx = Ptx / loss
        NS_LOG_INFO("Sivert Spectrum PSD Value after: " << *vit << std::endl);
//          NS_LOG_DEBUG("PSD gain received from Unity3D: " << psd_vector[RBnumber] << std::endl);
        totalRxPowerW += *vit * bwHz;
        ++vit;
        ++fit;
        
        }
      else{
          BandInfo fr = *fit;
          int RBnumber = vit - first_RB_index;
          *vit *= psd_vector[RBnumber];
          ++vit;
          ++fit;
        }

    }
    double totalRxPowerdBm = 10.0 * std::log10(totalRxPowerW * 1000.0);
    double avgRxPowerdBmPerSubChannel = 10.0 * std::log10((totalRxPowerW/psd_vector.size()) * 1000.0);
    NS_LOG_DEBUG(this << " Total power after SIVERT spectrum model: "<< totalRxPowerdBm << " avgRxPowerdBmPerSubChannel: " << avgRxPowerdBmPerSubChannel << " sub-channel BW is: "<< bwHz<<  std::endl);
//  std::cout<< "Exiting spectrum model..."<< std::endl;
  return rxPsd;
}

void
SivertSpectrumPropagationLossModel::SetChannelFromUnity3D(int TxID, int RxID, std::vector<double> gainVector)
{

  m_Channel_from_Unity3D[TxID][RxID] = gainVector;
  m_Channel_from_Unity3D[RxID][TxID] = gainVector;

}


std::vector<double> SivertSpectrumPropagationLossModel::GetChannelVectorFromUnity3D(int TxID, int RxID){
  return m_Channel_from_Unity3D[TxID][RxID];
}





}  // namespace ns3
