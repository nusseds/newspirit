package com.nusseds.newspirit

import android.os.Bundle
import androidx.preference.PreferenceFragmentCompat

class PreferenceFragment : PreferenceFragmentCompat() {
    override fun onCreatePreferences(savedInstanceState: Bundle?, rootKey: String?) {
        setPreferencesFromResource(R.xml.root_preferences, rootKey)
    }

}